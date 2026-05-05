from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import rclpy
from sensor_msgs.msg import Range

from sura_diagnostics.sensor_diagnostics_base import SensorDiagnosticsBase, finite_value


class DvlAltitudeDiagnostics(SensorDiagnosticsBase[Range]):
    def __init__(self) -> None:
        super().__init__(
            node_name="dvl_altitude_diagnostics",
            msg_type=Range,
            default_sensor_topic="/cirtesub/sensors/dvl/altitude_throttled",
            default_frequency_topic="/cirtesub/sensors/dvl/altitude",
            diagnostic_name="/Sensors/DVLAltitude",
            hardware_id="cirtesub_dvl_altitude",
            default_warn_min_frequency_hz=8.0,
            default_error_min_frequency_hz=2.0,
        )

    def _evaluate_message(self, msg: Range) -> tuple[int, str, list[KeyValue]]:
        altitude = float(msg.range)
        min_range = float(msg.min_range)
        max_range = float(msg.max_range)

        finite_altitude = finite_value(altitude)
        valid_bounds = math.isfinite(min_range) and math.isfinite(max_range)
        valid_altitude = finite_altitude and altitude >= 0.0
        inside_range = valid_bounds and min_range <= altitude <= max_range

        if not valid_altitude:
            level = DiagnosticStatus.ERROR
            message = "DVL altitude contains invalid range"
        elif not valid_bounds:
            level = DiagnosticStatus.ERROR
            message = "DVL altitude bounds are invalid"
        elif not inside_range:
            level = DiagnosticStatus.WARN
            message = "DVL altitude is outside reported bounds"
        else:
            level = DiagnosticStatus.OK
            message = "DVL altitude data healthy"

        values = [
            KeyValue(key="frame_id", value=msg.header.frame_id),
            KeyValue(key="altitude", value=f"{altitude:.6f}"),
            KeyValue(key="min_range", value=f"{min_range:.6f}"),
            KeyValue(key="max_range", value=f"{max_range:.6f}"),
            KeyValue(key="valid_altitude", value=str(valid_altitude).lower()),
            KeyValue(key="valid_bounds", value=str(valid_bounds).lower()),
            KeyValue(key="inside_range", value=str(inside_range).lower()),
        ]
        return level, message, values


def main() -> None:
    rclpy.init()
    node = DvlAltitudeDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
