from __future__ import annotations

from controller_manager_msgs.srv import ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State
from rclpy.node import Node
import rclpy


class HardwareComponentsDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("hardware_components_diagnostics")

        self.declare_parameter(
            "list_hardware_components_service",
            "/cirtesub/controller/controller_manager/list_hardware_components",
        )
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("service_timeout", 2.0)
        self.declare_parameter(
            "component_names",
            ["cirtesub_thrusters", "cirtesub/alpha_left", "cirtesub/alpha_right"],
        )
        self.declare_parameter(
            "diagnostic_names",
            [
                "/Hardware/Thrusters",
                "/Hardware/Left Arm",
                "/Hardware/Right Arm",
            ],
        )

        self._service_name = str(
            self.get_parameter("list_hardware_components_service").value
        )
        self._publish_period = float(self.get_parameter("publish_period").value)
        self._service_timeout = float(self.get_parameter("service_timeout").value)
        self._component_names = [
            str(name) for name in self.get_parameter("component_names").value
        ]
        self._diagnostic_names = [
            str(name) for name in self.get_parameter("diagnostic_names").value
        ]

        if len(self._diagnostic_names) != len(self._component_names):
            self.get_logger().warning(
                "diagnostic_names size does not match component_names; using component names"
            )
            self._diagnostic_names = [
                f"/Hardware/{name}" for name in self._component_names
            ]

        self._client = self.create_client(
            ListHardwareComponents,
            self._service_name,
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._pending_request = None
        self._request_start_seconds: float | None = None
        self._last_components = {}
        self._last_response_seconds: float | None = None

        self._timer = self.create_timer(self._publish_period, self._on_timer)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        now_seconds = now.nanoseconds / 1e9

        self._collect_response(now_seconds)
        self._publish_diagnostics(now, now_seconds)
        self._request_components(now_seconds)

    def _collect_response(self, now_seconds: float) -> None:
        if self._pending_request is None:
            return

        if self._pending_request.done():
            response = self._pending_request.result()
            self._pending_request = None
            self._request_start_seconds = None
            self._last_components = {
                component.name: component for component in response.component
            }
            self._last_response_seconds = now_seconds
            return

        if (
            self._request_start_seconds is not None
            and now_seconds - self._request_start_seconds > self._service_timeout
        ):
            self._pending_request = None
            self._request_start_seconds = None

    def _request_components(self, now_seconds: float) -> None:
        if self._pending_request is not None:
            return

        if not self._client.service_is_ready():
            return

        request = ListHardwareComponents.Request()
        self._pending_request = self._client.call_async(request)
        self._request_start_seconds = now_seconds

    def _publish_diagnostics(self, now, now_seconds: float) -> None:
        array_msg = DiagnosticArray()
        array_msg.header.stamp = now.to_msg()
        array_msg.status = [
            self._build_status(component_name, diagnostic_name, now_seconds)
            for component_name, diagnostic_name in zip(
                self._component_names,
                self._diagnostic_names,
            )
        ]
        self._diagnostics_pub.publish(array_msg)

    def _build_status(
        self,
        component_name: str,
        diagnostic_name: str,
        now_seconds: float,
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = diagnostic_name
        status.hardware_id = component_name

        component = self._last_components.get(component_name)
        age_seconds = None
        if self._last_response_seconds is not None:
            age_seconds = now_seconds - self._last_response_seconds

        if component is None:
            status.level = DiagnosticStatus.STALE
            status.message = "Hardware component state is unavailable"
            state_id = State.PRIMARY_STATE_UNKNOWN
            state_label = "unknown"
            component_type = "unknown"
            class_type = "unknown"
        else:
            state_id = component.state.id
            state_label = component.state.label
            component_type = component.type
            class_type = component.class_type
            status.level, status.message = self._evaluate_state(state_id, state_label)

        status.values = [
            KeyValue(key="component_name", value=component_name),
            KeyValue(key="state_id", value=str(state_id)),
            KeyValue(key="state_label", value=state_label),
            KeyValue(key="component_type", value=component_type),
            KeyValue(key="class_type", value=class_type),
            KeyValue(key="service", value=self._service_name),
            KeyValue(
                key="age_seconds",
                value="none" if age_seconds is None else f"{age_seconds:.3f}",
            ),
        ]
        return status

    def _evaluate_state(self, state_id: int, state_label: str) -> tuple[int, str]:
        normalized_label = state_label.lower()

        if state_id == State.PRIMARY_STATE_ACTIVE or normalized_label == "active":
            return DiagnosticStatus.OK, "Hardware component is active"

        if (
            state_id == State.PRIMARY_STATE_INACTIVE
            or normalized_label in ("inactive", "configured")
        ):
            return DiagnosticStatus.WARN, "Hardware component is configured but inactive"

        if (
            state_id in (State.PRIMARY_STATE_UNCONFIGURED, State.TRANSITION_STATE_CLEANINGUP)
            or normalized_label in ("unconfigured", "cleaningup", "cleanup")
        ):
            return DiagnosticStatus.ERROR, "Hardware component is in cleanup"

        return DiagnosticStatus.ERROR, f"Hardware component state is {state_label}"


def main() -> None:
    rclpy.init()
    node = HardwareComponentsDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
