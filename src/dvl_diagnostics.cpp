#include <algorithm>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/sensor_diagnostics_base.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using sura_diagnostics::SensorDiagnosticsBase;
using sura_diagnostics::bool_string;
using sura_diagnostics::covariance_diagonal_values;
using sura_diagnostics::finite_values;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

class DvlDiagnostics : public SensorDiagnosticsBase<TwistWithCovarianceStamped>
{
public:
  DvlDiagnostics()
  : SensorDiagnosticsBase(
      "dvl_diagnostics", "/cirtesub/sensors/dvl/twist_throttled",
      "/cirtesub/sensors/dvl/twist", "/Sensors/DVL", "cirtesub_dvl", 8.0, 2.0)
  {
  }

private:
  Evaluation evaluate_message(const TwistWithCovarianceStamped & msg) override
  {
    const auto & linear = msg.twist.twist.linear;
    const std::vector<double> linear_values = {linear.x, linear.y, linear.z};
    const auto covariance_diagonal = covariance_diagonal_values(msg.twist.covariance, 6);
    const std::vector<double> linear_covariance = {
      covariance_diagonal[0], covariance_diagonal[1], covariance_diagonal[2]};
    const auto finite_linear = finite_values(linear_values);
    const auto sane_covariance =
      finite_values(linear_covariance) &&
      std::all_of(linear_covariance.begin(), linear_covariance.end(), [](double value) {
        return value >= 0.0;
      });

    unsigned char level = DiagnosticStatus::OK;
    std::string message = "DVL twist data healthy";
    if (!finite_linear) {
      level = DiagnosticStatus::ERROR;
      message = "DVL twist contains invalid velocity";
    } else if (!sane_covariance) {
      level = DiagnosticStatus::ERROR;
      message = "DVL twist covariance is invalid";
    }

    std::vector<KeyValue> values = {
      key_value("frame_id", msg.header.frame_id),
      key_value("linear_x", fixed_string(linear_values[0], 6)),
      key_value("linear_y", fixed_string(linear_values[1], 6)),
      key_value("linear_z", fixed_string(linear_values[2], 6)),
      key_value("linear_covariance_x", fixed_string(linear_covariance[0], 6)),
      key_value("linear_covariance_y", fixed_string(linear_covariance[1], 6)),
      key_value("linear_covariance_z", fixed_string(linear_covariance[2], 6)),
      key_value("finite_velocity", bool_string(finite_linear)),
      key_value("sane_covariance", bool_string(sane_covariance)),
    };
    return {level, message, values};
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
