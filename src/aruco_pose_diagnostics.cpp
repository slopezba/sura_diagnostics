#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/sensor_diagnostics_base.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using sura_diagnostics::SensorDiagnosticsBase;
using sura_diagnostics::bool_string;
using sura_diagnostics::covariance_diagonal_values;
using sura_diagnostics::finite_values;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

class ArucoPoseDiagnostics : public SensorDiagnosticsBase<PoseWithCovarianceStamped>
{
public:
  ArucoPoseDiagnostics()
  : SensorDiagnosticsBase(
      "aruco_pose_diagnostics", "", "", "/Sensors/ArucoPose", "aruco_pose", 5.0, 1.0)
  {
    declare_parameter<double>("warn_min_quaternion_norm", 0.9);
    declare_parameter<double>("warn_max_quaternion_norm", 1.1);
    declare_parameter<double>("error_min_quaternion_norm", 0.5);
    declare_parameter<double>("error_max_quaternion_norm", 1.5);
  }

private:
  Evaluation evaluate_message(const PoseWithCovarianceStamped & msg) override
  {
    const auto & position = msg.pose.pose.position;
    const auto & orientation = msg.pose.pose.orientation;

    const std::vector<double> position_values = {
      position.x, position.y, position.z};
    const std::vector<double> orientation_values = {
      orientation.x, orientation.y, orientation.z, orientation.w};

    auto all_values = position_values;
    all_values.insert(
      all_values.end(),
      orientation_values.begin(),
      orientation_values.end());

    double quaternion_norm = 0.0;
    for (const auto value : orientation_values) {
      quaternion_norm += value * value;
    }
    quaternion_norm = std::sqrt(quaternion_norm);

    const auto finite_data = finite_values(all_values);
    const auto warn_min_norm = get_parameter("warn_min_quaternion_norm").as_double();
    const auto warn_max_norm = get_parameter("warn_max_quaternion_norm").as_double();
    const auto error_min_norm = get_parameter("error_min_quaternion_norm").as_double();
    const auto error_max_norm = get_parameter("error_max_quaternion_norm").as_double();
    const auto covariance_diagonal = covariance_diagonal_values(msg.pose.covariance, 6);
    const auto sane_covariance = finite_values(covariance_diagonal) &&
      std::all_of(covariance_diagonal.begin(), covariance_diagonal.end(), [](double value) {
        return value >= 0.0;
      });

    unsigned char level = DiagnosticStatus::OK;
    std::string message = "Aruco pose healthy";
    if (!finite_data) {
      level = DiagnosticStatus::ERROR;
      message = "Aruco pose contains invalid values";
    } else if (quaternion_norm < error_min_norm || quaternion_norm > error_max_norm) {
      level = DiagnosticStatus::ERROR;
      message = "Aruco pose quaternion norm is invalid";
    } else if (!sane_covariance) {
      level = DiagnosticStatus::ERROR;
      message = "Aruco pose covariance is invalid";
    } else if (quaternion_norm < warn_min_norm || quaternion_norm > warn_max_norm) {
      level = DiagnosticStatus::WARN;
      message = "Aruco pose quaternion norm is unusual";
    }

    std::vector<KeyValue> values = {
      key_value("frame_id", msg.header.frame_id),
      key_value("position_x", fixed_string(position.x, 6)),
      key_value("position_y", fixed_string(position.y, 6)),
      key_value("position_z", fixed_string(position.z, 6)),
      key_value("orientation_x", fixed_string(orientation.x, 6)),
      key_value("orientation_y", fixed_string(orientation.y, 6)),
      key_value("orientation_z", fixed_string(orientation.z, 6)),
      key_value("orientation_w", fixed_string(orientation.w, 6)),
      key_value("quaternion_norm", fixed_string(quaternion_norm, 6)),
      key_value("finite_data", bool_string(finite_data)),
      key_value("sane_covariance", bool_string(sane_covariance)),
    };

    return {level, message, values};
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoPoseDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
