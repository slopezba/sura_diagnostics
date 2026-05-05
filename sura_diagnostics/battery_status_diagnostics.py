from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import BatteryState


class BatteryStatusDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("battery_status_diagnostics")

        self.declare_parameter("battery_topic", "/cirtesub/battery/status")
        self.declare_parameter("publish_period", 2.0)
        self.declare_parameter("stale_timeout", 3.0)
        self.declare_parameter("warning_min_percentage", 0.25)
        self.declare_parameter("ok_min_percentage", 0.50)
        self.declare_parameter("warn_max_current_a", 60.0)
        self.declare_parameter("error_max_current_a", 90.0)

        self._battery_topic = str(self.get_parameter("battery_topic").value)
        self._publish_period = float(self.get_parameter("publish_period").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._warning_min_percentage = float(
            self.get_parameter("warning_min_percentage").value
        )
        self._ok_min_percentage = float(self.get_parameter("ok_min_percentage").value)
        self._warn_max_current_a = float(self.get_parameter("warn_max_current_a").value)
        self._error_max_current_a = float(self.get_parameter("error_max_current_a").value)

        self._last_battery_msg: BatteryState | None = None
        self._last_message_time: float | None = None

        self._subscription = self.create_subscription(
            BatteryState,
            self._battery_topic,
            self._on_battery,
            10,
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._timer = self.create_timer(self._publish_period, self._publish_diagnostics)

    def _on_battery(self, msg: BatteryState) -> None:
        self._last_battery_msg = msg
        self._last_message_time = self.get_clock().now().nanoseconds / 1e9

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        now_seconds = now.nanoseconds / 1e9

        status = DiagnosticStatus()
        status.name = "/Sensors/Battery"
        status.hardware_id = "cirtesub_battery"

        age_seconds = None
        if self._last_message_time is not None:
            age_seconds = now_seconds - self._last_message_time

        if (
            self._last_battery_msg is None
            or age_seconds is None
            or age_seconds > self._stale_timeout
        ):
            status.level = DiagnosticStatus.STALE
            status.message = "Battery status topic is stale"
            voltage = 0.0
            current_draw = 0.0
            percentage = 0.0
            present = False
        else:
            voltage = float(self._last_battery_msg.voltage)
            current_draw = abs(float(self._last_battery_msg.current))
            percentage = float(self._last_battery_msg.percentage)
            present = bool(self._last_battery_msg.present)
            status.level, status.message = self._evaluate_battery(
                percentage,
                current_draw,
                present,
            )

        status.values = [
            KeyValue(key="topic", value=self._battery_topic),
            KeyValue(key="voltage", value=f"{voltage:.3f}"),
            KeyValue(key="current_draw_a", value=f"{current_draw:.3f}"),
            KeyValue(key="percentage", value=f"{100.0 * percentage:.3f}"),
            KeyValue(key="present", value=str(present).lower()),
            KeyValue(key="ok_min_percentage", value=f"{100.0 * self._ok_min_percentage:.1f}"),
            KeyValue(
                key="warning_min_percentage",
                value=f"{100.0 * self._warning_min_percentage:.1f}",
            ),
            KeyValue(key="warn_max_current_a", value=f"{self._warn_max_current_a:.3f}"),
            KeyValue(key="error_max_current_a", value=f"{self._error_max_current_a:.3f}"),
            KeyValue(
                key="age_seconds",
                value="none" if age_seconds is None else f"{age_seconds:.3f}",
            ),
        ]

        array_msg = DiagnosticArray()
        array_msg.header.stamp = now.to_msg()
        array_msg.status = [status]
        self._diagnostics_pub.publish(array_msg)

    def _evaluate_battery(
        self,
        percentage: float,
        current_draw: float,
        present: bool,
    ) -> tuple[int, str]:
        if not present:
            return DiagnosticStatus.ERROR, "Battery is not present"

        if current_draw >= self._error_max_current_a:
            return DiagnosticStatus.ERROR, "Battery current draw is critical"
        if current_draw >= self._warn_max_current_a:
            return DiagnosticStatus.WARN, "Battery current draw is high"

        if percentage < self._ok_min_percentage:
            if percentage < self._warning_min_percentage:
                return (
                    DiagnosticStatus.ERROR,
                    f"Battery level critical: {100.0 * percentage:.3f}%",
                )
            return DiagnosticStatus.WARN, f"Battery level warning: {100.0 * percentage:.3f}%"

        return DiagnosticStatus.OK, f"Battery level ok: {100.0 * percentage:.3f}%"


def main() -> None:
    rclpy.init()
    node = BatteryStatusDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
