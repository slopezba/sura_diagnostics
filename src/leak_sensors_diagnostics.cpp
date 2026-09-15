#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"
#include "std_msgs/msg/bool.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using std_msgs::msg::Bool;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;
using sura_diagnostics::prefixed_diagnostic_name;

class LeakSensorsDiagnostics : public rclcpp::Node
{
public:
  LeakSensorsDiagnostics()
  : Node("leak_sensors_diagnostics")
  {
    declare_parameter<std::string>("leak_topic", "");
    declare_parameter<std::string>("hardware_id", "leak_sensors");
    declare_parameter<std::string>("robot_namespace", "");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("stale_timeout", 3.0);

    leak_topic_ = get_parameter("leak_topic").as_string();
    hardware_id_ = get_parameter("hardware_id").as_string();
    robot_namespace_ = get_parameter("robot_namespace").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();

    subscription_ = create_subscription<Bool>(
      leak_topic_, 10, [this](const Bool::SharedPtr msg) { on_leak_sensor(msg); });
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() { publish_diagnostics(); });
  }

private:
  void on_leak_sensor(const Bool::SharedPtr msg)
  {
    leak_detected_ = msg->data;
    has_state_ = true;
    last_update_seconds_ = now_seconds();
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    const auto seconds = time_seconds(now);
    const bool stale = !has_state_ || seconds - last_update_seconds_ > stale_timeout_;

    DiagnosticStatus status;
    status.name = prefixed_diagnostic_name(robot_namespace_, "/Sensors/LeakSensors");
    status.hardware_id = hardware_id_;
    if (stale) {
      status.level = DiagnosticStatus::STALE;
      status.message = "Leak sensor data is stale";
    } else if (leak_detected_) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Leak detected";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = "Leak sensors healthy";
    }

    status.values = {
      key_value("topic", leak_topic_),
      key_value("leak_detected", leak_detected_ ? "true" : "false"),
      key_value("has_state", has_state_ ? "true" : "false"),
      key_value("age_seconds", has_state_ ? fixed_string(seconds - last_update_seconds_, 3) : "nan"),
      key_value("stale_timeout", fixed_string(stale_timeout_, 3)),
    };

    DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    array_msg.status.push_back(status);
    diagnostics_pub_->publish(array_msg);
  }

  double now_seconds() { return time_seconds(get_clock()->now()); }
  static double time_seconds(const rclcpp::Time & time)
  {
    return static_cast<double>(time.nanoseconds()) / 1e9;
  }

  std::string leak_topic_;
  std::string hardware_id_{"leak_sensors"};
  std::string robot_namespace_;
  double publish_period_{1.0};
  double stale_timeout_{3.0};
  bool has_state_{false};
  bool leak_detected_{false};
  double last_update_seconds_{0.0};
  rclcpp::Subscription<Bool>::SharedPtr subscription_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeakSensorsDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
