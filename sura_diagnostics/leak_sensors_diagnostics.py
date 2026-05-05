from __future__ import annotations

from dataclasses import dataclass

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
import rclpy
from sura_msgs.msg import LeakSensor


@dataclass
class LeakSensorState:
    leak_detected: bool
    last_update_seconds: float


class LeakSensorsDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("leak_sensors_diagnostics")

        self.declare_parameter("leak_topic", "/cirtesub/leak_sensors")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("stale_timeout", 3.0)
        self.declare_parameter(
            "sensor_frames",
            ["cirtesub/main_cylinder", "cirtesub/battery_cylinder"],
        )

        self._leak_topic = str(self.get_parameter("leak_topic").value)
        self._publish_period = float(self.get_parameter("publish_period").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._states: dict[str, LeakSensorState] = {}

        self._subscription = self.create_subscription(
            LeakSensor,
            self._leak_topic,
            self._on_leak_sensor,
            10,
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._timer = self.create_timer(self._publish_period, self._publish_diagnostics)

    def _on_leak_sensor(self, msg: LeakSensor) -> None:
        frame = msg.frame_id.strip() or msg.header.frame_id.strip()
        if not frame:
            self.get_logger().warn("Received leak sensor message without frame_id")
            return

        self._states[frame] = LeakSensorState(
            leak_detected=bool(msg.leak_detected),
            last_update_seconds=self.get_clock().now().nanoseconds / 1e9,
        )

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        now_seconds = now.nanoseconds / 1e9
        expected_frames = self._sensor_frames()

        leaking_frames = []
        ok_frames = []
        stale_frames = []
        for frame in expected_frames:
            state = self._states.get(frame)
            if state is None or now_seconds - state.last_update_seconds > self._stale_timeout:
                stale_frames.append(frame)
            elif state.leak_detected:
                leaking_frames.append(frame)
            else:
                ok_frames.append(frame)

        status = DiagnosticStatus()
        status.name = "/Sensors/LeakSensors"
        status.hardware_id = "cirtesub_leak_sensors"

        if leaking_frames:
            status.level = DiagnosticStatus.ERROR
            status.message = "Leak detected"
        elif stale_frames:
            status.level = DiagnosticStatus.STALE
            status.message = "Leak sensor data is stale"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Leak sensors healthy"

        status.values = [
            KeyValue(key="topic", value=self._leak_topic),
            KeyValue(key="sensor_count", value=str(len(expected_frames))),
            KeyValue(key="ok_frames", value=",".join(ok_frames)),
            KeyValue(key="leaking_frames", value=",".join(leaking_frames)),
            KeyValue(key="stale_frames", value=",".join(stale_frames)),
            KeyValue(key="stale_timeout", value=f"{self._stale_timeout:.3f}"),
        ]

        array_msg = DiagnosticArray()
        array_msg.header.stamp = now.to_msg()
        array_msg.status = [status]
        self._diagnostics_pub.publish(array_msg)

    def _sensor_frames(self) -> list[str]:
        values = self.get_parameter("sensor_frames").get_parameter_value()
        frames = [str(frame) for frame in values.string_array_value]
        if frames:
            return frames
        return sorted(self._states.keys())


def main() -> None:
    rclpy.init()
    node = LeakSensorsDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
