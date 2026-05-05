from dataclasses import dataclass
from typing import Dict

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import rclpy
from sura_msgs.msg import ControllerDebug


@dataclass
class ControllerState:
    msg: ControllerDebug
    last_receive_time_ns: int


class ControllerDebugDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("controller_debug_diagnostics")
        self.declare_parameter("debug_topic", "/cirtesub/controller/debug")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("stale_timeout", 2.5)
        self.declare_parameter("controller_frequency_hz", 100.0)
        self.declare_parameter("use_controller_manager_update_rate", True)
        self.declare_parameter("controller_manager_node", "/cirtesub/controller/controller_manager")
        self.declare_parameter("update_rate_refresh_period", 5.0)
        self.declare_parameter("warning_time_margin_us", 2000.0)

        self._debug_topic = str(self.get_parameter("debug_topic").value)
        self._publish_period = float(self.get_parameter("publish_period").value)
        stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._stale_timeout = Duration(seconds=stale_timeout)
        self._controller_frequency_hz = float(
            self.get_parameter("controller_frequency_hz").value
        )
        self._controller_frequency_source = "configured"
        self._use_controller_manager_update_rate = bool(
            self.get_parameter("use_controller_manager_update_rate").value
        )
        self._controller_manager_node = str(
            self.get_parameter("controller_manager_node").value
        )
        self._update_rate_refresh_period = float(
            self.get_parameter("update_rate_refresh_period").value
        )
        self._warning_time_margin_us = float(
            self.get_parameter("warning_time_margin_us").value
        )
        self._states: Dict[str, ControllerState] = {}
        self._update_rate_request_in_flight = False

        self._subscription = self.create_subscription(
            ControllerDebug,
            self._debug_topic,
            self._on_controller_debug,
            10,
        )
        self._update_rate_client = self.create_client(
            GetParameters,
            f"{self._controller_manager_node}/get_parameters",
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._timer = self.create_timer(self._publish_period, self._publish_diagnostics)
        if self._use_controller_manager_update_rate:
            self._update_rate_timer = self.create_timer(
                self._update_rate_refresh_period,
                self._request_controller_manager_update_rate,
            )
            self._request_controller_manager_update_rate()
        else:
            self._update_rate_timer = None

    def _on_controller_debug(self, msg: ControllerDebug) -> None:
        controller_name = msg.controller_name.strip()
        if not controller_name:
            self.get_logger().warn("Received ControllerDebug message without controller_name")
            return

        self._states[controller_name] = ControllerState(
            msg=msg,
            last_receive_time_ns=self.get_clock().now().nanoseconds,
        )

    @staticmethod
    def _display_controller_name(controller_name: str) -> str:
        return controller_name.strip("/").split("/")[-1]

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        array_msg = DiagnosticArray()
        array_msg.header.stamp = now.to_msg()

        statuses = []
        total_deadline_misses = 0
        active_count = 0
        stale_count = 0
        worst_last_update_us = 0.0
        worst_max_update_us = 0.0
        worst_controller_name = ""
        summary_level = DiagnosticStatus.OK
        summary_message = "Controller diagnostics healthy"

        for controller_name in sorted(self._states.keys()):
            state = self._states[controller_name]
            status, is_stale = self._build_controller_status(controller_name, state, now)
            statuses.append(status)

            total_deadline_misses += int(state.msg.deadline_miss_count)
            if state.msg.active:
                active_count += 1
            if is_stale:
                stale_count += 1

            if state.msg.last_update_us >= worst_last_update_us:
                worst_last_update_us = float(state.msg.last_update_us)
                worst_controller_name = controller_name

            if state.msg.max_update_us >= worst_max_update_us:
                worst_max_update_us = float(state.msg.max_update_us)

            summary_level = max(summary_level, status.level)

        if not statuses:
            summary_level = DiagnosticStatus.WARN
            summary_message = "No controller debug data received"
        elif stale_count > 0:
            summary_message = "Some controller debug data is stale"
        elif total_deadline_misses > 0:
            summary_message = "Controllers reported deadline misses"
        elif active_count == 0:
            summary_message = "No active controllers reporting debug data"

        statuses.append(
            self._build_summary_status(
                level=summary_level,
                message=summary_message,
                total_count=len(self._states),
                active_count=active_count,
                stale_count=stale_count,
                total_deadline_misses=total_deadline_misses,
                worst_controller_name=worst_controller_name,
                worst_last_update_us=worst_last_update_us,
                worst_max_update_us=worst_max_update_us,
            )
        )

        array_msg.status = statuses
        self._diagnostics_pub.publish(array_msg)

    def _build_controller_status(
        self,
        controller_name: str,
        state: ControllerState,
        now,
    ) -> tuple[DiagnosticStatus, bool]:
        elapsed = now - Time(
            nanoseconds=state.last_receive_time_ns,
            clock_type=now.clock_type,
        )
        is_stale = elapsed > self._stale_timeout

        msg = state.msg
        display_controller_name = self._display_controller_name(controller_name)
        status = DiagnosticStatus()
        status.name = f"/Controllers/{display_controller_name}"
        status.hardware_id = "cirtesub_control_stack"

        configured_period_us = self._controller_period_us()
        reported_period_us = float(msg.desired_period_us)
        desired_period_us = reported_period_us if reported_period_us > 0.0 else configured_period_us
        warning_limit_us = max(0.0, desired_period_us - self._warning_time_margin_us)
        max_update_us = float(msg.max_update_us)
        margin_to_warning_us = warning_limit_us - max_update_us
        margin_to_error_us = desired_period_us - max_update_us

        if is_stale:
            status.level = DiagnosticStatus.STALE
            status.message = "Stale debug data"
        elif margin_to_error_us < 0.0 and desired_period_us > 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                "Update time exceeded configured period by "
                f"{abs(margin_to_error_us):.3f} us"
            )
        elif msg.deadline_miss_count > 0:
            status.level = DiagnosticStatus.WARN
            status.message = "Deadline misses detected"
        elif not msg.active:
            status.level = DiagnosticStatus.OK
            status.message = "Controller inactive"
        elif margin_to_warning_us <= 0.0:
            status.level = DiagnosticStatus.WARN
            status.message = (
                "Update time close to configured period: "
                f"{margin_to_error_us:.3f} us to error"
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                "Controller debug healthy: "
                f"{margin_to_warning_us:.3f} us to warning, "
                f"{margin_to_error_us:.3f} us to error"
            )

        status.values = [
            KeyValue(key="active", value=str(bool(msg.active)).lower()),
            KeyValue(key="chained_mode", value=str(bool(msg.chained_mode)).lower()),
            KeyValue(key="debug_topic", value=self._debug_topic),
            KeyValue(key="controller_frequency_hz", value=f"{self._controller_frequency_hz:.3f}"),
            KeyValue(key="controller_frequency_source", value=self._controller_frequency_source),
            KeyValue(key="controller_manager_node", value=self._controller_manager_node),
            KeyValue(key="configured_period_us", value=f"{configured_period_us:.3f}"),
            KeyValue(key="reported_period_us", value=f"{reported_period_us:.3f}"),
            KeyValue(key="desired_period_us", value=f"{desired_period_us:.3f}"),
            KeyValue(key="warning_limit_us", value=f"{warning_limit_us:.3f}"),
            KeyValue(key="margin_to_warning_us", value=f"{margin_to_warning_us:.3f}"),
            KeyValue(key="margin_to_error_us", value=f"{margin_to_error_us:.3f}"),
            KeyValue(key="warning_time_margin_us", value=f"{self._warning_time_margin_us:.3f}"),
            KeyValue(key="last_update_us", value=f"{float(msg.last_update_us):.3f}"),
            KeyValue(key="avg_update_us", value=f"{float(msg.avg_update_us):.3f}"),
            KeyValue(key="max_update_us", value=f"{float(msg.max_update_us):.3f}"),
            KeyValue(key="min_update_us", value=f"{float(msg.min_update_us):.3f}"),
            KeyValue(key="deadline_miss_count", value=str(int(msg.deadline_miss_count))),
            KeyValue(key="cycle_count", value=str(int(msg.cycle_count))),
            KeyValue(
                key="stale_for_ms",
                value=f"{float(elapsed.nanoseconds) / 1e6:.3f}",
            ),
        ]

        return status, is_stale

    def _controller_period_us(self) -> float:
        if self._controller_frequency_hz <= 0.0:
            return 0.0
        return 1_000_000.0 / self._controller_frequency_hz

    def _request_controller_manager_update_rate(self) -> None:
        if self._update_rate_request_in_flight:
            return
        if not self._update_rate_client.service_is_ready():
            return

        request = GetParameters.Request()
        request.names = ["update_rate"]
        future = self._update_rate_client.call_async(request)
        future.add_done_callback(self._on_update_rate_response)
        self._update_rate_request_in_flight = True

    def _on_update_rate_response(self, future) -> None:
        self._update_rate_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to read controller_manager update_rate: {exc}")
            return

        if not response.values:
            return

        value = response.values[0]
        update_rate = 0.0
        if value.type == ParameterType.PARAMETER_INTEGER:
            update_rate = float(value.integer_value)
        elif value.type == ParameterType.PARAMETER_DOUBLE:
            update_rate = float(value.double_value)

        if update_rate > 0.0:
            self._controller_frequency_hz = update_rate
            self._controller_frequency_source = "controller_manager.update_rate"

    def _build_summary_status(
        self,
        *,
        level: int,
        message: str,
        total_count: int,
        active_count: int,
        stale_count: int,
        total_deadline_misses: int,
        worst_controller_name: str,
        worst_last_update_us: float,
        worst_max_update_us: float,
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "/Controllers/Summary"
        status.hardware_id = "cirtesub_control_stack"
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key="total_count", value=str(total_count)),
            KeyValue(key="active_count", value=str(active_count)),
            KeyValue(key="stale_count", value=str(stale_count)),
            KeyValue(key="total_deadline_miss_count", value=str(total_deadline_misses)),
            KeyValue(key="worst_controller_name", value=worst_controller_name),
            KeyValue(key="worst_last_update_us", value=f"{worst_last_update_us:.3f}"),
            KeyValue(key="worst_max_update_us", value=f"{worst_max_update_us:.3f}"),
        ]
        return status


def main() -> None:
    rclpy.init()
    node = ControllerDebugDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
