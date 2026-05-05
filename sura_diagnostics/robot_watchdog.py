from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
import rclpy
from std_srvs.srv import Trigger
from sura_msgs.msg import Navigator, RobotWatchdog as RobotWatchdogMsg


class RobotWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("robot_watchdog")

        self.declare_parameter("navigator_topic", "/cirtesub/navigator/navigation")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("warning_timeout", 3600.0)
        self.declare_parameter("error_timeout_after_warning", 60.0)
        self.declare_parameter("surface_z_threshold", 0.2)

        self._navigator_topic = str(self.get_parameter("navigator_topic").value)
        self._publish_period = float(self.get_parameter("publish_period").value)
        self._warning_timeout = float(self.get_parameter("warning_timeout").value)
        self._error_timeout_after_warning = float(
            self.get_parameter("error_timeout_after_warning").value
        )
        self._surface_z_threshold = float(
            self.get_parameter("surface_z_threshold").value
        )

        now = self.get_clock().now()
        self._architecture_start_time = now
        self._last_reset_time = now
        self._last_reset_reason = "startup"
        self._reset_count = 0
        self._surface_hold_active = False
        self._last_navigation_z: float | None = None

        self._subscription = self.create_subscription(
            Navigator,
            self._navigator_topic,
            self._on_navigator,
            10,
        )
        self._reset_service = self.create_service(
            Trigger,
            "~/reset",
            self._on_reset,
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._watchdog_pub = self.create_publisher(
            RobotWatchdogMsg,
            "safety/robot_watchdog",
            10,
        )
        self._timer = self.create_timer(self._publish_period, self._publish_diagnostics)

    def _on_navigator(self, msg: Navigator) -> None:
        self._last_navigation_z = float(msg.position.position.z)
        surface_hold_active = self._last_navigation_z < self._surface_z_threshold
        if surface_hold_active and not self._surface_hold_active:
            self._reset("surface")
        elif surface_hold_active:
            self._last_reset_time = self.get_clock().now()
        self._surface_hold_active = surface_hold_active

    def _on_reset(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._reset("service")
        response.success = True
        response.message = "Robot watchdog reset"
        return response

    def _reset(self, reason: str) -> None:
        self._last_reset_time = self.get_clock().now()
        self._last_reset_reason = reason
        self._reset_count += 1

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        watchdog_msg = self._build_watchdog_msg(now)
        status = self._build_diagnostic_status(watchdog_msg)

        array_msg = DiagnosticArray()
        array_msg.header.stamp = now.to_msg()
        array_msg.status = [status]
        self._watchdog_pub.publish(watchdog_msg)
        self._diagnostics_pub.publish(array_msg)

    def _build_watchdog_msg(self, now) -> RobotWatchdogMsg:
        if self._surface_hold_active:
            elapsed_seconds = 0.0
            uptime_seconds = 0.0
            last_reset_time = now
        else:
            elapsed_seconds = max(
                0.0,
                (now.nanoseconds - self._last_reset_time.nanoseconds) / 1e9,
            )
            uptime_seconds = elapsed_seconds
            last_reset_time = self._last_reset_time

        total_error_timeout = self._warning_timeout + self._error_timeout_after_warning
        remaining_to_warning = max(0.0, self._warning_timeout - elapsed_seconds)
        remaining_to_error = max(0.0, total_error_timeout - elapsed_seconds)

        if elapsed_seconds >= total_error_timeout:
            state = RobotWatchdogMsg.ERROR
            state_name = "ERROR"
        elif elapsed_seconds >= self._warning_timeout:
            state = RobotWatchdogMsg.WARN
            state_name = "WARN"
        else:
            state = RobotWatchdogMsg.OK
            state_name = "OK"

        msg = RobotWatchdogMsg()
        msg.header.stamp = now.to_msg()
        msg.state = state
        msg.state_name = state_name
        msg.architecture_start_time = self._architecture_start_time.to_msg()
        msg.uptime_seconds = uptime_seconds
        msg.last_reset_time = last_reset_time.to_msg()
        msg.elapsed_since_reset_seconds = elapsed_seconds
        msg.remaining_to_warning_seconds = remaining_to_warning
        msg.remaining_to_error_seconds = remaining_to_error
        msg.reset_count = self._reset_count
        msg.last_reset_reason = self._last_reset_reason
        msg.warning_timeout = self._warning_timeout
        msg.error_timeout_after_warning = self._error_timeout_after_warning
        return msg

    def _build_diagnostic_status(
        self,
        watchdog_msg: RobotWatchdogMsg,
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "/Watchdog/Robot"
        status.hardware_id = "sura_robot_watchdog"

        state = watchdog_msg.state_name
        if watchdog_msg.state == RobotWatchdogMsg.ERROR:
            status.level = DiagnosticStatus.ERROR
            status.message = "Watchdog error timeout reached"
        elif watchdog_msg.state == RobotWatchdogMsg.WARN:
            status.level = DiagnosticStatus.WARN
            status.message = (
                "Watchdog remaining time to error: "
                f"{watchdog_msg.remaining_to_error_seconds:.3f} s"
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = (
                "Watchdog remaining time to warning: "
                f"{watchdog_msg.remaining_to_warning_seconds:.3f} s"
            )

        status.values = [
            KeyValue(key="state", value=state),
            KeyValue(key="state_name", value=watchdog_msg.state_name),
            KeyValue(
                key="architecture_start_time",
                value=(
                    f"{watchdog_msg.architecture_start_time.sec}."
                    f"{watchdog_msg.architecture_start_time.nanosec:09d}"
                ),
            ),
            KeyValue(
                key="last_reset_time",
                value=(
                    f"{watchdog_msg.last_reset_time.sec}."
                    f"{watchdog_msg.last_reset_time.nanosec:09d}"
                ),
            ),
            KeyValue(key="uptime_seconds", value=f"{watchdog_msg.uptime_seconds:.3f}"),
            KeyValue(
                key="elapsed_since_reset_seconds",
                value=f"{watchdog_msg.elapsed_since_reset_seconds:.3f}",
            ),
            KeyValue(
                key="remaining_to_warning_seconds",
                value=f"{watchdog_msg.remaining_to_warning_seconds:.3f}",
            ),
            KeyValue(
                key="remaining_to_error_seconds",
                value=f"{watchdog_msg.remaining_to_error_seconds:.3f}",
            ),
            KeyValue(key="warning_timeout", value=f"{self._warning_timeout:.3f}"),
            KeyValue(
                key="error_timeout_after_warning",
                value=f"{self._error_timeout_after_warning:.3f}",
            ),
            KeyValue(key="reset_count", value=str(watchdog_msg.reset_count)),
            KeyValue(key="last_reset_reason", value=watchdog_msg.last_reset_reason),
        ]
        return status


def main() -> None:
    rclpy.init()
    node = RobotWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
