#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sura_diagnostics/sensor_diagnostics_base.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::FluidPressure;
using sura_diagnostics::SensorDiagnosticsBase;
using sura_diagnostics::bool_string;
using sura_diagnostics::finite_value;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

class PressureDiagnostics : public SensorDiagnosticsBase<FluidPressure>
{
public:
  PressureDiagnostics()
  : SensorDiagnosticsBase(
      "pressure_diagnostics", "/cirtesub/sensors/pressure_throttled",
      "/cirtesub/sensors/pressure", "/Sensors/Pressure", "cirtesub_pressure", 0.8, 0.2)
  {
    declare_parameter<double>("surface_pressure", 101325.0);
    declare_parameter<double>("fluid_density", 1000.0);
    declare_parameter<double>("gravity", 9.80665);
  }

private:
  Evaluation evaluate_message(const FluidPressure & msg) override
  {
    const auto pressure = static_cast<double>(msg.fluid_pressure);
    const auto variance = static_cast<double>(msg.variance);
    const auto density = get_parameter("fluid_density").as_double();
    const auto gravity = get_parameter("gravity").as_double();
    const auto surface_pressure = get_parameter("surface_pressure").as_double();

    const auto finite_pressure = finite_value(pressure);
    const auto valid_pressure = finite_pressure && pressure > 0.0;
    const auto valid_variance = variance == 0.0 || (std::isfinite(variance) && variance > 0.0);
    const auto valid_parameters = density > 0.0 && gravity > 0.0;
    const auto depth = valid_parameters && finite_pressure ?
      (pressure - surface_pressure) / (density * gravity) :
      std::numeric_limits<double>::quiet_NaN();

    unsigned char level = DiagnosticStatus::OK;
    std::string message = "Pressure data healthy";
    if (!valid_pressure) {
      level = DiagnosticStatus::ERROR;
      message = "Pressure data contains invalid pressure";
    } else if (!valid_variance) {
      level = DiagnosticStatus::ERROR;
      message = "Pressure variance is invalid";
    } else if (!valid_parameters) {
      level = DiagnosticStatus::ERROR;
      message = "Pressure conversion parameters are invalid";
    }

    std::vector<KeyValue> values = {
      key_value("fluid_pressure", fixed_string(pressure, 3)),
      key_value("variance", fixed_string(variance, 6)),
      key_value("surface_pressure", fixed_string(surface_pressure, 3)),
      key_value("estimated_depth", fixed_string(depth, 6)),
      key_value("valid_pressure", bool_string(valid_pressure)),
      key_value("valid_variance", bool_string(valid_variance)),
    };
    return {level, message, values};
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PressureDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
