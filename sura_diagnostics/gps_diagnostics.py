from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from sura_diagnostics.sensor_diagnostics_base import SensorDiagnosticsBase, finite_values


class GpsDiagnostics(SensorDiagnosticsBase[NavSatFix]):
    def __init__(self) -> None:
        super().__init__(
            node_name="gps_diagnostics",
            msg_type=NavSatFix,
            default_sensor_topic="/cirtesub/sensors/gps_throttled",
            default_frequency_topic="/cirtesub/sensors/gps",
            diagnostic_name="/Sensors/GPS",
            hardware_id="cirtesub_gps",
            default_warn_min_frequency_hz=0.8,
            default_error_min_frequency_hz=0.2,
        )

    def _evaluate_message(self, msg: NavSatFix) -> tuple[int, str, list[KeyValue]]:
        finite_fix = finite_values(
            [float(msg.latitude), float(msg.longitude), float(msg.altitude)]
        )
        valid_range = -90.0 <= msg.latitude <= 90.0 and -180.0 <= msg.longitude <= 180.0
        has_fix = msg.status.status >= NavSatStatus.STATUS_FIX

        if not finite_fix or not valid_range:
            level = DiagnosticStatus.ERROR
            message = "GPS fix contains invalid coordinates"
        elif not has_fix:
            level = DiagnosticStatus.WARN
            message = "GPS has no fix"
        else:
            level = DiagnosticStatus.OK
            message = "GPS data healthy"

        values = [
            KeyValue(key="latitude", value=f"{float(msg.latitude):.9f}"),
            KeyValue(key="longitude", value=f"{float(msg.longitude):.9f}"),
            KeyValue(key="altitude", value=f"{float(msg.altitude):.3f}"),
            KeyValue(key="status", value=str(int(msg.status.status))),
            KeyValue(key="service", value=str(int(msg.status.service))),
            KeyValue(key="has_fix", value=str(has_fix).lower()),
            KeyValue(key="valid_coordinates", value=str(finite_fix and valid_range).lower()),
        ]
        return level, message, values


def main() -> None:
    rclpy.init()
    node = GpsDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
