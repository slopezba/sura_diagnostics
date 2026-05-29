#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::BatteryState;
using sura_diagnostics::bool_string;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;
using sura_diagnostics::optional_fixed_string;

class BatteryStatusDiagnostics : public rclcpp::Node
{
public:
  BatteryStatusDiagnostics()
  : Node("battery_status_diagnostics")
  {
    declare_parameter<std::string>("battery_topic", "");
    declare_parameter<std::string>("hardware_id", "battery");
    declare_parameter<double>("publish_period", 2.0);
    declare_parameter<double>("stale_timeout", 3.0);
    declare_parameter<double>("warning_min_percentage", 0.25);
    declare_parameter<double>("ok_min_percentage", 0.50);
    declare_parameter<double>("warn_max_current_a", 60.0);
    declare_parameter<double>("error_max_current_a", 90.0);

    battery_topic_ = get_parameter("battery_topic").as_string();
    hardware_id_ = get_parameter("hardware_id").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();
    warning_min_percentage_ = get_parameter("warning_min_percentage").as_double();
    ok_min_percentage_ = get_parameter("ok_min_percentage").as_double();
    warn_max_current_a_ = get_parameter("warn_max_current_a").as_double();
    error_max_current_a_ = get_parameter("error_max_current_a").as_double();

    subscription_ = create_subscription<BatteryState>(
      battery_topic_, 10, [this](const BatteryState::SharedPtr msg) { on_battery(msg); });
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() { publish_diagnostics(); });
  }

private:
  void on_battery(const BatteryState::SharedPtr msg)
  {
    last_battery_msg_ = msg;
    last_message_time_ = now_seconds();
    has_last_message_time_ = true;
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    const auto seconds = time_seconds(now);
    const auto has_age = has_last_message_time_;
    const auto age_seconds = has_age ? seconds - last_message_time_ : 0.0;

    DiagnosticStatus status;
    status.name = "/Sensors/Battery";
    status.hardware_id = hardware_id_;

    double voltage = 0.0;
    double current_draw = 0.0;
    double percentage = 0.0;
    bool present = false;

    if (!last_battery_msg_ || !has_age || age_seconds > stale_timeout_) {
      status.level = DiagnosticStatus::STALE;
      status.message = "Battery status topic is stale";
    } else {
      voltage = last_battery_msg_->voltage;
      current_draw = std::abs(last_battery_msg_->current);
      percentage = last_battery_msg_->percentage;
      present = last_battery_msg_->present;
      auto evaluation = evaluate_battery(percentage, current_draw, present);
      status.level = evaluation.first;
      status.message = evaluation.second;
    }

    status.values = {
      key_value("topic", battery_topic_),
      key_value("voltage", fixed_string(voltage, 3)),
      key_value("current_draw_a", fixed_string(current_draw, 3)),
      key_value("percentage", fixed_string(100.0 * percentage, 3)),
      key_value("present", bool_string(present)),
      key_value("ok_min_percentage", fixed_string(100.0 * ok_min_percentage_, 1)),
      key_value("warning_min_percentage", fixed_string(100.0 * warning_min_percentage_, 1)),
      key_value("warn_max_current_a", fixed_string(warn_max_current_a_, 3)),
      key_value("error_max_current_a", fixed_string(error_max_current_a_, 3)),
      key_value("age_seconds", optional_fixed_string(has_age, age_seconds, 3)),
    };

    DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    array_msg.status.push_back(status);
    diagnostics_pub_->publish(array_msg);
  }

  std::pair<unsigned char, std::string> evaluate_battery(
    double percentage, double current_draw, bool present) const
  {
    if (!present) {
      return {DiagnosticStatus::ERROR, "Battery is not present"};
    }
    if (current_draw >= error_max_current_a_) {
      return {DiagnosticStatus::ERROR, "Battery current draw is critical"};
    }
    if (current_draw >= warn_max_current_a_) {
      return {DiagnosticStatus::WARN, "Battery current draw is high"};
    }
    if (percentage < ok_min_percentage_) {
      if (percentage < warning_min_percentage_) {
        return {
          DiagnosticStatus::ERROR,
          "Battery level critical: " + fixed_string(100.0 * percentage, 3) + "%"};
      }
      return {
        DiagnosticStatus::WARN,
        "Battery level warning: " + fixed_string(100.0 * percentage, 3) + "%"};
    }
    return {DiagnosticStatus::OK, "Battery level ok: " + fixed_string(100.0 * percentage, 3) + "%"};
  }

  double now_seconds() { return time_seconds(get_clock()->now()); }
  static double time_seconds(const rclcpp::Time & time)
  {
    return static_cast<double>(time.nanoseconds()) / 1e9;
  }

  std::string battery_topic_;
  std::string hardware_id_{"battery"};
  double publish_period_{2.0};
  double stale_timeout_{3.0};
  double warning_min_percentage_{0.25};
  double ok_min_percentage_{0.50};
  double warn_max_current_a_{60.0};
  double error_max_current_a_{90.0};
  BatteryState::SharedPtr last_battery_msg_;
  bool has_last_message_time_{false};
  double last_message_time_{0.0};
  rclcpp::Subscription<BatteryState>::SharedPtr subscription_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStatusDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
