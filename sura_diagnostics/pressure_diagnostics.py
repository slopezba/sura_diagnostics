from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import rclpy
from sensor_msgs.msg import FluidPressure

from sura_diagnostics.sensor_diagnostics_base import SensorDiagnosticsBase, finite_value


class PressureDiagnostics(SensorDiagnosticsBase[FluidPressure]):
    def __init__(self) -> None:
        super().__init__(
            node_name="pressure_diagnostics",
            msg_type=FluidPressure,
            default_sensor_topic="/cirtesub/sensors/pressure_throttled",
            default_frequency_topic="/cirtesub/sensors/pressure",
            diagnostic_name="/Sensors/Pressure",
            hardware_id="cirtesub_pressure",
            default_warn_min_frequency_hz=0.8,
            default_error_min_frequency_hz=0.2,
        )
        self.declare_parameter("surface_pressure", 101325.0)
        self.declare_parameter("fluid_density", 1000.0)
        self.declare_parameter("gravity", 9.80665)

    def _evaluate_message(
        self,
        msg: FluidPressure,
    ) -> tuple[int, str, list[KeyValue]]:
        pressure = float(msg.fluid_pressure)
        variance = float(msg.variance)
        density = float(self.get_parameter("fluid_density").value)
        gravity = float(self.get_parameter("gravity").value)
        surface_pressure = float(self.get_parameter("surface_pressure").value)

        finite_pressure = finite_value(pressure)
        valid_pressure = finite_pressure and pressure > 0.0
        valid_variance = variance == 0.0 or (math.isfinite(variance) and variance > 0.0)
        valid_parameters = density > 0.0 and gravity > 0.0
        if valid_parameters and finite_pressure:
            depth = (pressure - surface_pressure) / (density * gravity)
        else:
            depth = math.nan

        if not valid_pressure:
            level = DiagnosticStatus.ERROR
            message = "Pressure data contains invalid pressure"
        elif not valid_variance:
            level = DiagnosticStatus.ERROR
            message = "Pressure variance is invalid"
        elif not valid_parameters:
            level = DiagnosticStatus.ERROR
            message = "Pressure conversion parameters are invalid"
        else:
            level = DiagnosticStatus.OK
            message = "Pressure data healthy"

        values = [
            KeyValue(key="fluid_pressure", value=f"{pressure:.3f}"),
            KeyValue(key="variance", value=f"{variance:.6f}"),
            KeyValue(key="surface_pressure", value=f"{surface_pressure:.3f}"),
            KeyValue(key="estimated_depth", value=f"{depth:.6f}"),
            KeyValue(key="valid_pressure", value=str(valid_pressure).lower()),
            KeyValue(key="valid_variance", value=str(valid_variance).lower()),
        ]
        return level, message, values


def main() -> None:
    rclpy.init()
    node = PressureDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
