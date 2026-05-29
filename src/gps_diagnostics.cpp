#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sura_diagnostics/sensor_diagnostics_base.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using sura_diagnostics::SensorDiagnosticsBase;
using sura_diagnostics::bool_string;
using sura_diagnostics::finite_values;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

class GpsDiagnostics : public SensorDiagnosticsBase<NavSatFix>
{
public:
  GpsDiagnostics()
  : SensorDiagnosticsBase(
      "gps_diagnostics", "", "", "/Sensors/GPS", "gps", 0.8, 0.2)
  {
  }

private:
  Evaluation evaluate_message(const NavSatFix & msg) override
  {
    const auto finite_fix = finite_values({msg.latitude, msg.longitude, msg.altitude});
    const auto valid_range =
      -90.0 <= msg.latitude && msg.latitude <= 90.0 &&
      -180.0 <= msg.longitude && msg.longitude <= 180.0;
    const auto has_fix = msg.status.status >= NavSatStatus::STATUS_FIX;

    unsigned char level = DiagnosticStatus::OK;
    std::string message = "GPS data healthy";
    if (!finite_fix || !valid_range) {
      level = DiagnosticStatus::ERROR;
      message = "GPS fix contains invalid coordinates";
    } else if (!has_fix) {
      level = DiagnosticStatus::WARN;
      message = "GPS has no fix";
    }

    std::vector<KeyValue> values = {
      key_value("latitude", fixed_string(msg.latitude, 9)),
      key_value("longitude", fixed_string(msg.longitude, 9)),
      key_value("altitude", fixed_string(msg.altitude, 3)),
      key_value("status", std::to_string(static_cast<int>(msg.status.status))),
      key_value("service", std::to_string(static_cast<int>(msg.status.service))),
      key_value("has_fix", bool_string(has_fix)),
      key_value("valid_coordinates", bool_string(finite_fix && valid_range)),
    };
    return {level, message, values};
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
