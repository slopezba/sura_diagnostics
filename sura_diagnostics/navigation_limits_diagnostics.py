from __future__ import annotations

from collections import deque
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
import rclpy
from sura_msgs.msg import NavigationSafety, Navigator


class NavigationLimitsDiagnostics(Node):
    def __init__(self) -> None:
        super().__init__("navigation_limits_diagnostics")

        self.declare_parameter("navigator_topic", "/cirtesub/navigator/navigation_throttled")
        self.declare_parameter("frequency_topic", "/cirtesub/navigator/navigation")
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("stale_timeout", 3.0)
        self.declare_parameter("warn_min_frequency_hz", 10.0)
        self.declare_parameter("error_min_frequency_hz", 2.0)
        self.declare_parameter("max_depth", 5.0)
        self.declare_parameter("depth_warning_margin", 0.5)
        self.declare_parameter("min_altitude", 0.5)
        self.declare_parameter("altitude_warning_margin", 0.5)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("max_radius", 20.0)
        self.declare_parameter("radius_warning_margin", 2.0)

        self._navigator_topic = str(self.get_parameter("navigator_topic").value)
        self._frequency_topic = str(self.get_parameter("frequency_topic").value)
        self._publish_period = float(self.get_parameter("publish_period").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._warn_min_frequency_hz = float(
            self.get_parameter("warn_min_frequency_hz").value
        )
        self._error_min_frequency_hz = float(
            self.get_parameter("error_min_frequency_hz").value
        )
        self._max_depth = float(self.get_parameter("max_depth").value)
        self._depth_warning_margin = float(
            self.get_parameter("depth_warning_margin").value
        )
        self._min_altitude = float(self.get_parameter("min_altitude").value)
        self._altitude_warning_margin = float(
            self.get_parameter("altitude_warning_margin").value
        )
        self._center_x = float(self.get_parameter("center_x").value)
        self._center_y = float(self.get_parameter("center_y").value)
        self._max_radius = float(self.get_parameter("max_radius").value)
        self._radius_warning_margin = float(
            self.get_parameter("radius_warning_margin").value
        )

        self._last_navigation_msg: Navigator | None = None
        self._last_message_time: float | None = None
        self._frequency_timestamps = deque()
        self._last_frequency_message_time: float | None = None

        self._subscription = self.create_subscription(
            Navigator,
            self._navigator_topic,
            self._on_navigator,
            10,
        )
        self._frequency_subscription = self.create_subscription(
            Navigator,
            self._frequency_topic,
            self._on_frequency_navigator,
            10,
        )
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._safety_pub = self.create_publisher(
            NavigationSafety,
            "safety/navigation",
            10,
        )
        self._timer = self.create_timer(self._publish_period, self._publish_diagnostics)

    def _on_navigator(self, msg: Navigator) -> None:
        self._last_navigation_msg = msg
        self._last_message_time = self.get_clock().now().nanoseconds / 1e9

    def _on_frequency_navigator(self, _msg: Navigator) -> None:
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        self._last_frequency_message_time = now_seconds
        self._frequency_timestamps.append(now_seconds)
        self._drop_old_frequency_timestamps(now_seconds)

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now()
        now_seconds = now.nanoseconds / 1e9
        self._drop_old_frequency_timestamps(now_seconds)
        age_seconds = None
        if self._last_message_time is not None:
            age_seconds = now_seconds - self._last_message_time
        frequency_age_seconds = None
        if self._last_frequency_message_time is not None:
            frequency_age_seconds = now_seconds - self._last_frequency_message_time

        if (
            self._last_navigation_msg is None
            or age_seconds is None
            or age_seconds > self._stale_timeout
        ):
            depth_status = self._build_stale_status(
                "/Navigation/DepthLimit",
                age_seconds,
            )
            altitude_status = self._build_stale_status(
                "/Navigation/AltitudeLimit",
                age_seconds,
            )
            area_status = self._build_stale_status(
                "/Navigation/AreaLimit",
                age_seconds,
            )
        else:
            depth_status = self._build_depth_status(self._last_navigation_msg, age_seconds)
            altitude_status = self._build_altitude_status(
                self._last_navigation_msg,
                age_seconds,
            )
            area_status = self._build_area_status(self._last_navigation_msg, age_seconds)

        safety_msg = self._build_safety_msg(
            now,
            age_seconds,
            frequency_age_seconds,
        )
        frequency_status = self._build_frequency_status(
            safety_msg,
            frequency_age_seconds,
        )

        array_msg = DiagnosticArray()
        array_msg.header.stamp = now.to_msg()
        array_msg.status = [
            depth_status,
            altitude_status,
            area_status,
            frequency_status,
        ]
        self._safety_pub.publish(safety_msg)
        self._diagnostics_pub.publish(array_msg)

    def _build_safety_msg(
        self,
        now,
        age_seconds: float | None,
        frequency_age_seconds: float | None,
    ) -> NavigationSafety:
        msg = NavigationSafety()
        msg.header.stamp = now.to_msg()

        if (
            self._last_navigation_msg is None
            or age_seconds is None
            or age_seconds > self._stale_timeout
        ):
            msg.depth_state = NavigationSafety.STALE
            msg.depth = math.nan
            msg.depth_remaining_to_warning = math.nan
            msg.depth_remaining_to_error = math.nan
            msg.area_state = NavigationSafety.STALE
            msg.area_distance = math.nan
            msg.area_remaining_to_warning = math.nan
            msg.area_remaining_to_error = math.nan
        else:
            current_depth = float(self._last_navigation_msg.position.position.z)
            depth_warn_limit = self._max_depth - self._depth_warning_margin
            msg.depth_state = self._state_from_limits(
                current_depth,
                depth_warn_limit,
                self._max_depth,
            )
            msg.depth = current_depth
            msg.depth_remaining_to_warning = depth_warn_limit - current_depth
            msg.depth_remaining_to_error = self._max_depth - current_depth

            current_x = float(self._last_navigation_msg.position.position.x)
            current_y = float(self._last_navigation_msg.position.position.y)
            area_distance = math.hypot(
                current_x - self._center_x,
                current_y - self._center_y,
            )
            area_warn_limit = self._max_radius - self._radius_warning_margin
            msg.area_state = self._state_from_limits(
                area_distance,
                area_warn_limit,
                self._max_radius,
            )
            msg.area_distance = area_distance
            msg.area_remaining_to_warning = area_warn_limit - area_distance
            msg.area_remaining_to_error = self._max_radius - area_distance

        msg.depth_limit = self._max_depth
        msg.area_limit = self._max_radius

        frequency_hz = self._frequency_hz()
        msg.frequency_hz = frequency_hz
        msg.frequency_warn_limit = self._warn_min_frequency_hz
        msg.frequency_error_limit = self._error_min_frequency_hz
        msg.frequency_remaining_to_warning = frequency_hz - self._warn_min_frequency_hz
        msg.frequency_remaining_to_error = frequency_hz - self._error_min_frequency_hz
        if frequency_age_seconds is None or frequency_age_seconds > self._stale_timeout:
            msg.frequency_state = NavigationSafety.STALE
        elif frequency_hz < self._error_min_frequency_hz:
            msg.frequency_state = NavigationSafety.ERROR
        elif frequency_hz < self._warn_min_frequency_hz:
            msg.frequency_state = NavigationSafety.WARN
        else:
            msg.frequency_state = NavigationSafety.OK

        return msg

    def _build_stale_status(
        self,
        name: str,
        age_seconds: float | None,
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = "sura_navigation_limits"
        status.level = DiagnosticStatus.STALE
        status.message = "Navigation limits input is stale"
        status.values = self._base_values(age_seconds)
        return status

    def _build_frequency_status(
        self,
        safety_msg: NavigationSafety,
        age_seconds: float | None,
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "/Navigation/Frequency"
        status.hardware_id = "sura_navigation"

        if safety_msg.frequency_state == NavigationSafety.STALE:
            status.level = DiagnosticStatus.STALE
            status.message = "Navigator topic is stale"
        elif safety_msg.frequency_state == NavigationSafety.ERROR:
            status.level = DiagnosticStatus.ERROR
            status.message = "Navigator frequency is critically low"
        elif safety_msg.frequency_state == NavigationSafety.WARN:
            status.level = DiagnosticStatus.WARN
            status.message = "Navigator frequency is low"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Navigator frequency healthy"

        status.values = [
            KeyValue(key="topic", value=self._frequency_topic),
            KeyValue(key="frequency_hz", value=f"{safety_msg.frequency_hz:.3f}"),
            KeyValue(key="sample_count", value=str(len(self._frequency_timestamps))),
            KeyValue(key="window_seconds", value=f"{self._publish_period:.3f}"),
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
        ]
        return status

    def _build_depth_status(
        self,
        msg: Navigator,
        age_seconds: float,
    ) -> DiagnosticStatus:
        current_depth = float(msg.position.position.z)
        remaining_depth = self._max_depth - current_depth

        status = DiagnosticStatus()
        status.name = "/Navigation/DepthLimit"
        status.hardware_id = "sura_navigation_limits"

        if remaining_depth < 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = "Maximum depth exceeded"
        elif remaining_depth <= self._depth_warning_margin:
            status.level = DiagnosticStatus.WARN
            status.message = "Approaching maximum depth"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Depth inside limits"

        status.values = self._base_values(age_seconds) + [
            KeyValue(key="current_depth", value=f"{current_depth:.3f}"),
            KeyValue(key="max_depth", value=f"{self._max_depth:.3f}"),
            KeyValue(key="remaining_depth_margin", value=f"{remaining_depth:.3f}"),
            KeyValue(
                key="depth_warning_margin",
                value=f"{self._depth_warning_margin:.3f}",
            ),
        ]
        return status

    def _build_altitude_status(
        self,
        msg: Navigator,
        age_seconds: float,
    ) -> DiagnosticStatus:
        current_altitude = float(msg.altitude)
        remaining_altitude = current_altitude - self._min_altitude

        status = DiagnosticStatus()
        status.name = "/Navigation/AltitudeLimit"
        status.hardware_id = "sura_navigation_limits"

        if not math.isfinite(current_altitude) or current_altitude < 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = "Altitude data is invalid"
        elif remaining_altitude < 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = "Minimum altitude exceeded"
        elif remaining_altitude <= self._altitude_warning_margin:
            status.level = DiagnosticStatus.WARN
            status.message = "Approaching minimum altitude"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Altitude inside limits"

        status.values = self._base_values(age_seconds) + [
            KeyValue(key="current_altitude", value=f"{current_altitude:.3f}"),
            KeyValue(key="min_altitude", value=f"{self._min_altitude:.3f}"),
            KeyValue(
                key="remaining_altitude_margin",
                value=f"{remaining_altitude:.3f}",
            ),
            KeyValue(
                key="altitude_warning_margin",
                value=f"{self._altitude_warning_margin:.3f}",
            ),
        ]
        return status

    def _build_area_status(
        self,
        msg: Navigator,
        age_seconds: float,
    ) -> DiagnosticStatus:
        current_x = float(msg.position.position.x)
        current_y = float(msg.position.position.y)
        dx = current_x - self._center_x
        dy = current_y - self._center_y
        distance_xy = math.hypot(dx, dy)
        remaining_radius = self._max_radius - distance_xy

        status = DiagnosticStatus()
        status.name = "/Navigation/AreaLimit"
        status.hardware_id = "sura_navigation_limits"

        if remaining_radius < 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = "Maximum navigation radius exceeded"
        elif remaining_radius <= self._radius_warning_margin:
            status.level = DiagnosticStatus.WARN
            status.message = "Approaching maximum navigation radius"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Navigation area inside limits"

        status.values = self._base_values(age_seconds) + [
            KeyValue(key="current_x", value=f"{current_x:.3f}"),
            KeyValue(key="current_y", value=f"{current_y:.3f}"),
            KeyValue(key="center_x", value=f"{self._center_x:.3f}"),
            KeyValue(key="center_y", value=f"{self._center_y:.3f}"),
            KeyValue(key="distance_xy", value=f"{distance_xy:.3f}"),
            KeyValue(key="max_radius", value=f"{self._max_radius:.3f}"),
            KeyValue(key="remaining_radius_margin", value=f"{remaining_radius:.3f}"),
            KeyValue(
                key="radius_warning_margin",
                value=f"{self._radius_warning_margin:.3f}",
            ),
        ]
        return status

    def _base_values(self, age_seconds: float | None) -> list[KeyValue]:
        return [
            KeyValue(key="topic", value=self._navigator_topic),
            KeyValue(
                key="age_seconds",
                value="none" if age_seconds is None else f"{age_seconds:.3f}",
            ),
            KeyValue(key="stale_timeout", value=f"{self._stale_timeout:.3f}"),
        ]

    def _drop_old_frequency_timestamps(self, now_seconds: float) -> None:
        while (
            self._frequency_timestamps
            and now_seconds - self._frequency_timestamps[0] > self._publish_period
        ):
            self._frequency_timestamps.popleft()

    def _frequency_hz(self) -> float:
        if len(self._frequency_timestamps) < 2:
            return 0.0

        elapsed = self._frequency_timestamps[-1] - self._frequency_timestamps[0]
        if elapsed <= 0.0:
            return 0.0

        return float(len(self._frequency_timestamps) - 1) / elapsed

    def _state_from_limits(
        self,
        value: float,
        warning_limit: float,
        error_limit: float,
    ) -> int:
        if value > error_limit:
            return NavigationSafety.ERROR
        if value >= warning_limit:
            return NavigationSafety.WARN
        return NavigationSafety.OK


def main() -> None:
    rclpy.init()
    node = NavigationLimitsDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
