from __future__ import annotations

from collections import deque
import math
from typing import Generic, TypeVar

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


MsgT = TypeVar("MsgT")


class SensorDiagnosticsBase(Node, Generic[MsgT]):
    def __init__(
        self,
        node_name: str,
        msg_type,
        default_sensor_topic: str,
        default_frequency_topic: str,
        diagnostic_name: str,
        hardware_id: str,
        default_warn_min_frequency_hz: float,
        default_error_min_frequency_hz: float,
    ) -> None:
        super().__init__(node_name)

        self.declare_parameter("sensor_topic", default_sensor_topic)
        self.declare_parameter("frequency_topic", default_frequency_topic)
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("stale_timeout", 3.0)
        self.declare_parameter("frequency_window", 5.0)
        self.declare_parameter("warn_min_frequency_hz", default_warn_min_frequency_hz)
        self.declare_parameter("error_min_frequency_hz", default_error_min_frequency_hz)

        self._sensor_topic = str(self.get_parameter("sensor_topic").value)
        self._frequency_topic = str(self.get_parameter("frequency_topic").value)
        self._publish_period = float(self.get_parameter("publish_period").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._frequency_window = float(self.get_parameter("frequency_window").value)
        self._warn_min_frequency_hz = float(
            self.get_parameter("warn_min_frequency_hz").value
        )
        self._error_min_frequency_hz = float(
            self.get_parameter("error_min_frequency_hz").value
        )
        self._diagnostic_name = diagnostic_name
        self._hardware_id = hardware_id

        self._last_sensor_msg: MsgT | None = None
        self._last_message_time: float | None = None
        self._last_frequency_message_time: float | None = None
        self._frequency_timestamps = deque()

        self._subscription = self.create_subscription(
            msg_type,
            self._sensor_topic,
            self._on_sensor_msg,
            10,
        )
        self._frequency_subscription = self.create_subscription(
            msg_type,
            self._frequency_topic,
            self._on_frequency_msg,
            10,
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._timer = self.create_timer(self._publish_period, self._publish_diagnostics)

    def _on_sensor_msg(self, msg: MsgT) -> None:
        self._last_sensor_msg = msg
        self._last_message_time = self.get_clock().now().nanoseconds / 1e9

    def _on_frequency_msg(self, _msg: MsgT) -> None:
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        self._last_frequency_message_time = now_seconds
        self._frequency_timestamps.append(now_seconds)
        self._drop_old_frequency_timestamps(now_seconds)

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        now_seconds = now.nanoseconds / 1e9
        self._drop_old_frequency_timestamps(now_seconds)

        status = self._build_status(now_seconds)
        array_msg = DiagnosticArray()
        array_msg.header.stamp = now.to_msg()
        array_msg.status = [status]
        self._diagnostics_pub.publish(array_msg)

    def _build_status(self, now_seconds: float) -> DiagnosticStatus:
        age_seconds = self._age_seconds(now_seconds)
        frequency_age_seconds = self._frequency_age_seconds(now_seconds)
        frequency_hz = self._frequency_hz()

        status = DiagnosticStatus()
        status.name = self._diagnostic_name
        status.hardware_id = self._hardware_id

        if (
            self._last_sensor_msg is None
            or age_seconds is None
            or age_seconds > self._stale_timeout
        ):
            status.level = DiagnosticStatus.STALE
            status.message = "Sensor data is stale"
            validation_values = self._stale_values()
        else:
            status.level, status.message, validation_values = self._evaluate_message(
                self._last_sensor_msg
            )

        if (
            frequency_age_seconds is None
            or frequency_age_seconds > self._stale_timeout
        ):
            self._apply_frequency_level(
                status,
                DiagnosticStatus.STALE,
                "Sensor frequency topic is stale",
            )
        elif frequency_hz < self._error_min_frequency_hz:
            self._apply_frequency_level(
                status,
                DiagnosticStatus.ERROR,
                "Sensor frequency is critically low",
            )
        elif frequency_hz < self._warn_min_frequency_hz:
            self._apply_frequency_level(
                status,
                DiagnosticStatus.WARN,
                "Sensor frequency is low",
            )

        status.values = self._base_values(
            age_seconds,
            frequency_age_seconds,
            frequency_hz,
        ) + validation_values
        return status

    def _apply_frequency_level(
        self,
        status: DiagnosticStatus,
        level: int,
        message: str,
    ) -> None:
        if level > status.level:
            status.level = level
            status.message = message

    def _evaluate_message(self, msg: MsgT) -> tuple[int, str, list[KeyValue]]:
        raise NotImplementedError

    def _stale_values(self) -> list[KeyValue]:
        return []

    def _base_values(
        self,
        age_seconds: float | None,
        frequency_age_seconds: float | None,
        frequency_hz: float,
    ) -> list[KeyValue]:
        return [
            KeyValue(key="sensor_topic", value=self._sensor_topic),
            KeyValue(key="frequency_topic", value=self._frequency_topic),
            KeyValue(key="frequency_hz", value=f"{frequency_hz:.3f}"),
            KeyValue(key="sample_count", value=str(len(self._frequency_timestamps))),
            KeyValue(key="frequency_window", value=f"{self._frequency_window:.3f}"),
            KeyValue(
                key="warn_min_frequency_hz",
                value=f"{self._warn_min_frequency_hz:.3f}",
            ),
            KeyValue(
                key="error_min_frequency_hz",
                value=f"{self._error_min_frequency_hz:.3f}",
            ),
            KeyValue(
                key="age_seconds",
                value="none" if age_seconds is None else f"{age_seconds:.3f}",
            ),
            KeyValue(
                key="frequency_age_seconds",
                value=(
                    "none"
                    if frequency_age_seconds is None
                    else f"{frequency_age_seconds:.3f}"
                ),
            ),
            KeyValue(key="stale_timeout", value=f"{self._stale_timeout:.3f}"),
        ]

    def _age_seconds(self, now_seconds: float) -> float | None:
        if self._last_message_time is None:
            return None
        return now_seconds - self._last_message_time

    def _frequency_age_seconds(self, now_seconds: float) -> float | None:
        if self._last_frequency_message_time is None:
            return None
        return now_seconds - self._last_frequency_message_time

    def _drop_old_frequency_timestamps(self, now_seconds: float) -> None:
        while (
            self._frequency_timestamps
            and now_seconds - self._frequency_timestamps[0] > self._frequency_window
        ):
            self._frequency_timestamps.popleft()

    def _frequency_hz(self) -> float:
        if len(self._frequency_timestamps) < 2:
            return 0.0

        elapsed = self._frequency_timestamps[-1] - self._frequency_timestamps[0]
        if elapsed <= 0.0:
            return 0.0

        return float(len(self._frequency_timestamps) - 1) / elapsed


def finite_value(value: float) -> bool:
    return math.isfinite(float(value))


def finite_values(values: list[float]) -> bool:
    return all(finite_value(value) for value in values)


def covariance_diagonal_values(covariance: list[float], size: int) -> list[float]:
    return [float(covariance[index * size + index]) for index in range(size)]
