#include <algorithm>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "sura_msgs/msg/robot_watchdog.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using std_srvs::srv::Trigger;
using sura_msgs::msg::Navigator;
using sura_msgs::msg::RobotWatchdog;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

class RobotWatchdogNode : public rclcpp::Node
{
public:
  RobotWatchdogNode()
  : Node("robot_watchdog")
  {
    declare_parameter<std::string>("navigator_topic", "/cirtesub/navigator/navigation");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("warning_timeout", 3600.0);
    declare_parameter<double>("error_timeout_after_warning", 60.0);
    declare_parameter<double>("surface_z_threshold", 0.2);

    navigator_topic_ = get_parameter("navigator_topic").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    warning_timeout_ = get_parameter("warning_timeout").as_double();
    error_timeout_after_warning_ = get_parameter("error_timeout_after_warning").as_double();
    surface_z_threshold_ = get_parameter("surface_z_threshold").as_double();

    const auto now = get_clock()->now();
    architecture_start_time_ = now;
    last_reset_time_ = now;

    subscription_ = create_subscription<Navigator>(
      navigator_topic_, 10, [this](const Navigator::SharedPtr msg) { on_navigator(msg); });
    reset_service_ = create_service<Trigger>(
      "~/reset",
      [this](
        const std::shared_ptr<Trigger::Request> request,
        const std::shared_ptr<Trigger::Response> response) { on_reset(request, response); });
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    watchdog_pub_ = create_publisher<RobotWatchdog>("safety/robot_watchdog", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() { publish_diagnostics(); });
  }

private:
  void on_navigator(const Navigator::SharedPtr msg)
  {
    last_navigation_z_ = msg->position.position.z;
    has_last_navigation_z_ = true;
    const auto surface_hold_active = last_navigation_z_ < surface_z_threshold_;
    if (surface_hold_active && !surface_hold_active_) {
      reset("surface");
    } else if (surface_hold_active) {
      last_reset_time_ = get_clock()->now();
    }
    surface_hold_active_ = surface_hold_active;
  }

  void on_reset(
    const std::shared_ptr<Trigger::Request>,
    const std::shared_ptr<Trigger::Response> response)
  {
    reset("service");
    response->success = true;
    response->message = "Robot watchdog reset";
  }

  void reset(const std::string & reason)
  {
    last_reset_time_ = get_clock()->now();
    last_reset_reason_ = reason;
    ++reset_count_;
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    const auto watchdog_msg = build_watchdog_msg(now);
    const auto status = build_diagnostic_status(watchdog_msg);

    DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    array_msg.status.push_back(status);
    watchdog_pub_->publish(watchdog_msg);
    diagnostics_pub_->publish(array_msg);
  }

  RobotWatchdog build_watchdog_msg(const rclcpp::Time & now) const
  {
    double elapsed_seconds = 0.0;
    double uptime_seconds = 0.0;
    auto last_reset_time = last_reset_time_;
    if (surface_hold_active_) {
      last_reset_time = now;
    } else {
      elapsed_seconds = std::max(0.0, static_cast<double>((now - last_reset_time_).nanoseconds()) / 1e9);
      uptime_seconds = elapsed_seconds;
    }

    const auto total_error_timeout = warning_timeout_ + error_timeout_after_warning_;
    const auto remaining_to_warning = std::max(0.0, warning_timeout_ - elapsed_seconds);
    const auto remaining_to_error = std::max(0.0, total_error_timeout - elapsed_seconds);

    unsigned char state = RobotWatchdog::OK;
    std::string state_name = "OK";
    if (elapsed_seconds >= total_error_timeout) {
      state = RobotWatchdog::ERROR;
      state_name = "ERROR";
    } else if (elapsed_seconds >= warning_timeout_) {
      state = RobotWatchdog::WARN;
      state_name = "WARN";
    }

    RobotWatchdog msg;
    msg.header.stamp = now;
    msg.state = state;
    msg.state_name = state_name;
    msg.architecture_start_time = architecture_start_time_;
    msg.uptime_seconds = uptime_seconds;
    msg.last_reset_time = last_reset_time;
    msg.elapsed_since_reset_seconds = elapsed_seconds;
    msg.remaining_to_warning_seconds = remaining_to_warning;
    msg.remaining_to_error_seconds = remaining_to_error;
    msg.reset_count = reset_count_;
    msg.last_reset_reason = last_reset_reason_;
    msg.warning_timeout = warning_timeout_;
    msg.error_timeout_after_warning = error_timeout_after_warning_;
    return msg;
  }

  DiagnosticStatus build_diagnostic_status(const RobotWatchdog & watchdog_msg) const
  {
    DiagnosticStatus status;
    status.name = "/Watchdog/Robot";
    status.hardware_id = "sura_robot_watchdog";

    if (watchdog_msg.state == RobotWatchdog::ERROR) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Watchdog error timeout reached";
    } else if (watchdog_msg.state == RobotWatchdog::WARN) {
      status.level = DiagnosticStatus::WARN;
      status.message =
        "Watchdog remaining time to error: " +
        fixed_string(watchdog_msg.remaining_to_error_seconds, 3) + " s";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message =
        "Watchdog remaining time to warning: " +
        fixed_string(watchdog_msg.remaining_to_warning_seconds, 3) + " s";
    }

    status.values = {
      key_value("state", watchdog_msg.state_name),
      key_value("state_name", watchdog_msg.state_name),
      key_value("architecture_start_time", time_stamp_string(watchdog_msg.architecture_start_time)),
      key_value("last_reset_time", time_stamp_string(watchdog_msg.last_reset_time)),
      key_value("uptime_seconds", fixed_string(watchdog_msg.uptime_seconds, 3)),
      key_value(
        "elapsed_since_reset_seconds",
        fixed_string(watchdog_msg.elapsed_since_reset_seconds, 3)),
      key_value(
        "remaining_to_warning_seconds",
        fixed_string(watchdog_msg.remaining_to_warning_seconds, 3)),
      key_value(
        "remaining_to_error_seconds",
        fixed_string(watchdog_msg.remaining_to_error_seconds, 3)),
      key_value("warning_timeout", fixed_string(warning_timeout_, 3)),
      key_value("error_timeout_after_warning", fixed_string(error_timeout_after_warning_, 3)),
      key_value("reset_count", std::to_string(watchdog_msg.reset_count)),
      key_value("last_reset_reason", watchdog_msg.last_reset_reason),
    };
    return status;
  }

  static std::string time_stamp_string(const builtin_interfaces::msg::Time & stamp)
  {
    return std::to_string(stamp.sec) + "." + padded_nanoseconds(stamp.nanosec);
  }

  static std::string padded_nanoseconds(uint32_t nanoseconds)
  {
    auto value = std::to_string(nanoseconds);
    if (value.size() < 9) {
      value.insert(value.begin(), 9 - value.size(), '0');
    }
    return value;
  }

  std::string navigator_topic_;
  double publish_period_{1.0};
  double warning_timeout_{3600.0};
  double error_timeout_after_warning_{60.0};
  double surface_z_threshold_{0.2};
  rclcpp::Time architecture_start_time_;
  rclcpp::Time last_reset_time_;
  std::string last_reset_reason_{"startup"};
  uint64_t reset_count_{0};
  bool surface_hold_active_{false};
  bool has_last_navigation_z_{false};
  double last_navigation_z_{0.0};
  rclcpp::Subscription<Navigator>::SharedPtr subscription_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<RobotWatchdog>::SharedPtr watchdog_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotWatchdogNode>());
  rclcpp::shutdown();
  return 0;
}
