from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy

from sura_diagnostics.sensor_diagnostics_base import (
    SensorDiagnosticsBase,
    covariance_diagonal_values,
    finite_values,
)


class DvlDiagnostics(SensorDiagnosticsBase[TwistWithCovarianceStamped]):
    def __init__(self) -> None:
        super().__init__(
            node_name="dvl_diagnostics",
            msg_type=TwistWithCovarianceStamped,
            default_sensor_topic="/cirtesub/sensors/dvl/twist_throttled",
            default_frequency_topic="/cirtesub/sensors/dvl/twist",
            diagnostic_name="/Sensors/DVL",
            hardware_id="cirtesub_dvl",
            default_warn_min_frequency_hz=8.0,
            default_error_min_frequency_hz=2.0,
        )

    def _evaluate_message(
        self,
        msg: TwistWithCovarianceStamped,
    ) -> tuple[int, str, list[KeyValue]]:
        linear = msg.twist.twist.linear
        linear_values = [float(linear.x), float(linear.y), float(linear.z)]
        covariance_diagonal = covariance_diagonal_values(list(msg.twist.covariance), 6)
        linear_covariance = [covariance_diagonal[index] for index in (0, 1, 2)]
        finite_linear = finite_values(linear_values)
        sane_covariance = finite_values(linear_covariance) and all(
            value >= 0.0 for value in linear_covariance
        )

        if not finite_linear:
            level = DiagnosticStatus.ERROR
            message = "DVL twist contains invalid velocity"
        elif not sane_covariance:
            level = DiagnosticStatus.ERROR
            message = "DVL twist covariance is invalid"
        else:
            level = DiagnosticStatus.OK
            message = "DVL twist data healthy"

        values = [
            KeyValue(key="frame_id", value=msg.header.frame_id),
            KeyValue(key="linear_x", value=f"{linear_values[0]:.6f}"),
            KeyValue(key="linear_y", value=f"{linear_values[1]:.6f}"),
            KeyValue(key="linear_z", value=f"{linear_values[2]:.6f}"),
            KeyValue(key="linear_covariance_x", value=f"{linear_covariance[0]:.6f}"),
            KeyValue(key="linear_covariance_y", value=f"{linear_covariance[1]:.6f}"),
            KeyValue(key="linear_covariance_z", value=f"{linear_covariance[2]:.6f}"),
            KeyValue(key="finite_velocity", value=str(finite_linear).lower()),
            KeyValue(key="sane_covariance", value=str(sane_covariance).lower()),
        ]
        return level, message, values


def main() -> None:
    rclpy.init()
    node = DvlDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
