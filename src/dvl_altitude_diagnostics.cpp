#include <cmath>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sura_diagnostics/sensor_diagnostics_base.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::Range;
using sura_diagnostics::SensorDiagnosticsBase;
using sura_diagnostics::bool_string;
using sura_diagnostics::finite_value;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

class DvlAltitudeDiagnostics : public SensorDiagnosticsBase<Range>
{
public:
  DvlAltitudeDiagnostics()
  : SensorDiagnosticsBase(
      "dvl_altitude_diagnostics", "", "", "/Sensors/DVLAltitude", "dvl_altitude", 8.0, 2.0)
  {
  }

private:
  Evaluation evaluate_message(const Range & msg) override
  {
    const auto altitude = static_cast<double>(msg.range);
    const auto min_range = static_cast<double>(msg.min_range);
    const auto max_range = static_cast<double>(msg.max_range);
    const auto finite_altitude = finite_value(altitude);
    const auto valid_bounds = std::isfinite(min_range) && std::isfinite(max_range);
    const auto valid_altitude = finite_altitude && altitude >= 0.0;
    const auto inside_range = valid_bounds && min_range <= altitude && altitude <= max_range;

    unsigned char level = DiagnosticStatus::OK;
    std::string message = "DVL altitude data healthy";
    if (!valid_altitude) {
      level = DiagnosticStatus::ERROR;
      message = "DVL altitude contains invalid range";
    } else if (!valid_bounds) {
      level = DiagnosticStatus::ERROR;
      message = "DVL altitude bounds are invalid";
    } else if (!inside_range) {
      level = DiagnosticStatus::WARN;
      message = "DVL altitude is outside reported bounds";
    }

    std::vector<KeyValue> values = {
      key_value("frame_id", msg.header.frame_id),
      key_value("altitude", fixed_string(altitude, 6)),
      key_value("min_range", fixed_string(min_range, 6)),
      key_value("max_range", fixed_string(max_range, 6)),
      key_value("valid_altitude", bool_string(valid_altitude)),
      key_value("valid_bounds", bool_string(valid_bounds)),
      key_value("inside_range", bool_string(inside_range)),
    };
    return {level, message, values};
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlAltitudeDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
