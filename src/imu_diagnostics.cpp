#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sura_diagnostics/sensor_diagnostics_base.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::Imu;
using sura_diagnostics::SensorDiagnosticsBase;
using sura_diagnostics::bool_string;
using sura_diagnostics::covariance_diagonal_values;
using sura_diagnostics::finite_values;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

class ImuDiagnostics : public SensorDiagnosticsBase<Imu>
{
public:
  ImuDiagnostics()
  : SensorDiagnosticsBase(
      "imu_diagnostics", "/cirtesub/sensors/imu_throttled", "/cirtesub/sensors/imu",
      "/Sensors/IMU", "cirtesub_imu", 40.0, 10.0)
  {
    declare_parameter<double>("warn_min_quaternion_norm", 0.9);
    declare_parameter<double>("warn_max_quaternion_norm", 1.1);
    declare_parameter<double>("error_min_quaternion_norm", 0.5);
    declare_parameter<double>("error_max_quaternion_norm", 1.5);
  }

private:
  Evaluation evaluate_message(const Imu & msg) override
  {
    const auto & orientation = msg.orientation;
    const auto & angular_velocity = msg.angular_velocity;
    const auto & linear_acceleration = msg.linear_acceleration;
    const std::vector<double> orientation_values = {
      orientation.x, orientation.y, orientation.z, orientation.w};
    const std::vector<double> angular_values = {
      angular_velocity.x, angular_velocity.y, angular_velocity.z};
    const std::vector<double> acceleration_values = {
      linear_acceleration.x, linear_acceleration.y, linear_acceleration.z};

    auto all_values = orientation_values;
    all_values.insert(all_values.end(), angular_values.begin(), angular_values.end());
    all_values.insert(all_values.end(), acceleration_values.begin(), acceleration_values.end());

    double quaternion_norm = 0.0;
    for (const auto value : orientation_values) {
      quaternion_norm += value * value;
    }
    quaternion_norm = std::sqrt(quaternion_norm);

    const auto finite_data = finite_values(all_values);
    const auto sane_covariance =
      is_covariance_sane(msg.orientation_covariance) &&
      is_covariance_sane(msg.angular_velocity_covariance) &&
      is_covariance_sane(msg.linear_acceleration_covariance);

    const auto warn_min_norm = get_parameter("warn_min_quaternion_norm").as_double();
    const auto warn_max_norm = get_parameter("warn_max_quaternion_norm").as_double();
    const auto error_min_norm = get_parameter("error_min_quaternion_norm").as_double();
    const auto error_max_norm = get_parameter("error_max_quaternion_norm").as_double();

    unsigned char level = DiagnosticStatus::OK;
    std::string message = "IMU data healthy";
    if (!finite_data) {
      level = DiagnosticStatus::ERROR;
      message = "IMU data contains invalid values";
    } else if (quaternion_norm < error_min_norm || quaternion_norm > error_max_norm) {
      level = DiagnosticStatus::ERROR;
      message = "IMU orientation quaternion norm is invalid";
    } else if (!sane_covariance) {
      level = DiagnosticStatus::ERROR;
      message = "IMU covariance is invalid";
    } else if (quaternion_norm < warn_min_norm || quaternion_norm > warn_max_norm) {
      level = DiagnosticStatus::WARN;
      message = "IMU orientation quaternion norm is unusual";
    }

    std::vector<KeyValue> values = {
      key_value("frame_id", msg.header.frame_id),
      key_value("quaternion_norm", fixed_string(quaternion_norm, 6)),
      key_value("angular_velocity_x", fixed_string(angular_values[0], 6)),
      key_value("angular_velocity_y", fixed_string(angular_values[1], 6)),
      key_value("angular_velocity_z", fixed_string(angular_values[2], 6)),
      key_value("linear_acceleration_x", fixed_string(acceleration_values[0], 6)),
      key_value("linear_acceleration_y", fixed_string(acceleration_values[1], 6)),
      key_value("linear_acceleration_z", fixed_string(acceleration_values[2], 6)),
      key_value("finite_data", bool_string(finite_data)),
      key_value("sane_covariance", bool_string(sane_covariance)),
    };
    return {level, message, values};
  }

  template<typename ArrayT>
  bool is_covariance_sane(const ArrayT & covariance) const
  {
    const auto diagonal = covariance_diagonal_values(covariance, 3);
    if (!finite_values(diagonal)) {
      return false;
    }
    if (covariance[0] == -1.0) {
      return true;
    }
    return std::all_of(diagonal.begin(), diagonal.end(), [](double value) {
      return value >= 0.0;
    });
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
