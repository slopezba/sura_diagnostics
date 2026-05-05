from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import rclpy
from sensor_msgs.msg import Imu

from sura_diagnostics.sensor_diagnostics_base import (
    SensorDiagnosticsBase,
    covariance_diagonal_values,
    finite_values,
)


class ImuDiagnostics(SensorDiagnosticsBase[Imu]):
    def __init__(self) -> None:
        super().__init__(
            node_name="imu_diagnostics",
            msg_type=Imu,
            default_sensor_topic="/cirtesub/sensors/imu_throttled",
            default_frequency_topic="/cirtesub/sensors/imu",
            diagnostic_name="/Sensors/IMU",
            hardware_id="cirtesub_imu",
            default_warn_min_frequency_hz=40.0,
            default_error_min_frequency_hz=10.0,
        )
        self.declare_parameter("warn_min_quaternion_norm", 0.9)
        self.declare_parameter("warn_max_quaternion_norm", 1.1)
        self.declare_parameter("error_min_quaternion_norm", 0.5)
        self.declare_parameter("error_max_quaternion_norm", 1.5)

    def _evaluate_message(self, msg: Imu) -> tuple[int, str, list[KeyValue]]:
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        orientation_values = [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ]
        angular_values = [
            float(angular_velocity.x),
            float(angular_velocity.y),
            float(angular_velocity.z),
        ]
        acceleration_values = [
            float(linear_acceleration.x),
            float(linear_acceleration.y),
            float(linear_acceleration.z),
        ]
        quaternion_norm = math.sqrt(sum(value * value for value in orientation_values))

        finite_data = finite_values(
            orientation_values + angular_values + acceleration_values
        )
        sane_covariance = (
            self._is_covariance_sane(list(msg.orientation_covariance))
            and self._is_covariance_sane(list(msg.angular_velocity_covariance))
            and self._is_covariance_sane(list(msg.linear_acceleration_covariance))
        )

        warn_min_norm = float(self.get_parameter("warn_min_quaternion_norm").value)
        warn_max_norm = float(self.get_parameter("warn_max_quaternion_norm").value)
        error_min_norm = float(self.get_parameter("error_min_quaternion_norm").value)
        error_max_norm = float(self.get_parameter("error_max_quaternion_norm").value)

        if not finite_data:
            level = DiagnosticStatus.ERROR
            message = "IMU data contains invalid values"
        elif quaternion_norm < error_min_norm or quaternion_norm > error_max_norm:
            level = DiagnosticStatus.ERROR
            message = "IMU orientation quaternion norm is invalid"
        elif not sane_covariance:
            level = DiagnosticStatus.ERROR
            message = "IMU covariance is invalid"
        elif quaternion_norm < warn_min_norm or quaternion_norm > warn_max_norm:
            level = DiagnosticStatus.WARN
            message = "IMU orientation quaternion norm is unusual"
        else:
            level = DiagnosticStatus.OK
            message = "IMU data healthy"

        values = [
            KeyValue(key="frame_id", value=msg.header.frame_id),
            KeyValue(key="quaternion_norm", value=f"{quaternion_norm:.6f}"),
            KeyValue(key="angular_velocity_x", value=f"{angular_values[0]:.6f}"),
            KeyValue(key="angular_velocity_y", value=f"{angular_values[1]:.6f}"),
            KeyValue(key="angular_velocity_z", value=f"{angular_values[2]:.6f}"),
            KeyValue(key="linear_acceleration_x", value=f"{acceleration_values[0]:.6f}"),
            KeyValue(key="linear_acceleration_y", value=f"{acceleration_values[1]:.6f}"),
            KeyValue(key="linear_acceleration_z", value=f"{acceleration_values[2]:.6f}"),
            KeyValue(key="finite_data", value=str(finite_data).lower()),
            KeyValue(key="sane_covariance", value=str(sane_covariance).lower()),
        ]
        return level, message, values

    def _is_covariance_sane(self, covariance: list[float]) -> bool:
        diagonal = covariance_diagonal_values(covariance, 3)
        if not finite_values(diagonal):
            return False
        if covariance[0] == -1.0:
            return True
        return all(value >= 0.0 for value in diagonal)


def main() -> None:
    rclpy.init()
    node = ImuDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
