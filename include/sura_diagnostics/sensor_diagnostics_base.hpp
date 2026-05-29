#pragma once

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"

namespace sura_diagnostics
{

template<typename MsgT>
class SensorDiagnosticsBase : public rclcpp::Node
{
public:
  using Evaluation = std::tuple<unsigned char, std::string, std::vector<diagnostic_msgs::msg::KeyValue>>;

  SensorDiagnosticsBase(
    const std::string & node_name,
    const std::string & default_sensor_topic,
    const std::string & default_frequency_topic,
    const std::string & diagnostic_name,
    const std::string & hardware_id,
    double default_warn_min_frequency_hz,
    double default_error_min_frequency_hz)
  : rclcpp::Node(node_name),
    diagnostic_name_(diagnostic_name),
    hardware_id_(hardware_id)
  {
    declare_parameter<std::string>("sensor_topic", default_sensor_topic);
    declare_parameter<std::string>("frequency_topic", default_frequency_topic);
    declare_parameter<std::string>("hardware_id", hardware_id);
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("stale_timeout", 3.0);
    declare_parameter<double>("frequency_window", 5.0);
    declare_parameter<double>("warn_min_frequency_hz", default_warn_min_frequency_hz);
    declare_parameter<double>("error_min_frequency_hz", default_error_min_frequency_hz);

    sensor_topic_ = get_parameter("sensor_topic").as_string();
    frequency_topic_ = get_parameter("frequency_topic").as_string();
    hardware_id_ = get_parameter("hardware_id").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();
    frequency_window_ = get_parameter("frequency_window").as_double();
    warn_min_frequency_hz_ = get_parameter("warn_min_frequency_hz").as_double();
    error_min_frequency_hz_ = get_parameter("error_min_frequency_hz").as_double();

    subscription_ = create_subscription<MsgT>(
      sensor_topic_, 10, [this](const typename MsgT::SharedPtr msg) { on_sensor_msg(msg); });
    frequency_subscription_ = create_subscription<MsgT>(
      frequency_topic_, 10, [this](const typename MsgT::SharedPtr msg) { on_frequency_msg(msg); });
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_),
      [this]() { publish_diagnostics(); });
  }

protected:
  virtual Evaluation evaluate_message(const MsgT & msg) = 0;

  virtual std::vector<diagnostic_msgs::msg::KeyValue> stale_values()
  {
    return {};
  }

private:
  void on_sensor_msg(const typename MsgT::SharedPtr msg)
  {
    last_sensor_msg_ = msg;
    last_message_time_ = now_seconds();
    has_last_message_time_ = true;
  }

  void on_frequency_msg(const typename MsgT::SharedPtr)
  {
    const auto seconds = now_seconds();
    last_frequency_message_time_ = seconds;
    has_last_frequency_message_time_ = true;
    frequency_timestamps_.push_back(seconds);
    drop_old_frequency_timestamps(seconds);
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    const auto seconds = time_seconds(now);
    drop_old_frequency_timestamps(seconds);

    diagnostic_msgs::msg::DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    array_msg.status.push_back(build_status(seconds));
    diagnostics_pub_->publish(array_msg);
  }

  diagnostic_msgs::msg::DiagnosticStatus build_status(double seconds)
  {
    const auto age = age_seconds(seconds);
    const auto frequency_age = frequency_age_seconds(seconds);
    const auto frequency = frequency_hz();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = diagnostic_name_;
    status.hardware_id = hardware_id_;

    std::vector<diagnostic_msgs::msg::KeyValue> validation_values;
    if (!last_sensor_msg_ || !age.first || age.second > stale_timeout_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "Sensor data is stale";
      validation_values = stale_values();
    } else {
      auto [level, message, values] = evaluate_message(*last_sensor_msg_);
      status.level = level;
      status.message = message;
      validation_values = std::move(values);
    }

    if (!frequency_age.first || frequency_age.second > stale_timeout_) {
      apply_frequency_level(
        status, diagnostic_msgs::msg::DiagnosticStatus::STALE,
        "Sensor frequency topic is stale");
    } else if (frequency < error_min_frequency_hz_) {
      apply_frequency_level(
        status, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Sensor frequency is critically low");
    } else if (frequency < warn_min_frequency_hz_) {
      apply_frequency_level(
        status, diagnostic_msgs::msg::DiagnosticStatus::WARN, "Sensor frequency is low");
    }

    status.values = base_values(age, frequency_age, frequency);
    status.values.insert(status.values.end(), validation_values.begin(), validation_values.end());
    return status;
  }

  void apply_frequency_level(
    diagnostic_msgs::msg::DiagnosticStatus & status, unsigned char level,
    const std::string & message)
  {
    if (level > status.level) {
      status.level = level;
      status.message = message;
    }
  }

  std::vector<diagnostic_msgs::msg::KeyValue> base_values(
    const std::pair<bool, double> & age,
    const std::pair<bool, double> & frequency_age,
    double frequency) const
  {
    return {
      key_value("sensor_topic", sensor_topic_),
      key_value("frequency_topic", frequency_topic_),
      key_value("frequency_hz", fixed_string(frequency, 3)),
      key_value("sample_count", std::to_string(frequency_timestamps_.size())),
      key_value("frequency_window", fixed_string(frequency_window_, 3)),
      key_value("warn_min_frequency_hz", fixed_string(warn_min_frequency_hz_, 3)),
      key_value("error_min_frequency_hz", fixed_string(error_min_frequency_hz_, 3)),
      key_value("age_seconds", optional_fixed_string(age.first, age.second, 3)),
      key_value(
        "frequency_age_seconds",
        optional_fixed_string(frequency_age.first, frequency_age.second, 3)),
      key_value("stale_timeout", fixed_string(stale_timeout_, 3)),
    };
  }

  std::pair<bool, double> age_seconds(double seconds) const
  {
    if (!has_last_message_time_) {
      return {false, 0.0};
    }
    return {true, seconds - last_message_time_};
  }

  std::pair<bool, double> frequency_age_seconds(double seconds) const
  {
    if (!has_last_frequency_message_time_) {
      return {false, 0.0};
    }
    return {true, seconds - last_frequency_message_time_};
  }

  void drop_old_frequency_timestamps(double seconds)
  {
    while (!frequency_timestamps_.empty() &&
      seconds - frequency_timestamps_.front() > frequency_window_)
    {
      frequency_timestamps_.pop_front();
    }
  }

  double frequency_hz() const
  {
    if (frequency_timestamps_.size() < 2) {
      return 0.0;
    }
    const auto elapsed = frequency_timestamps_.back() - frequency_timestamps_.front();
    if (elapsed <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(frequency_timestamps_.size() - 1) / elapsed;
  }

  double now_seconds()
  {
    return time_seconds(get_clock()->now());
  }

  static double time_seconds(const rclcpp::Time & time)
  {
    return static_cast<double>(time.nanoseconds()) / 1e9;
  }

  std::string sensor_topic_;
  std::string frequency_topic_;
  double publish_period_{1.0};
  double stale_timeout_{3.0};
  double frequency_window_{5.0};
  double warn_min_frequency_hz_{0.0};
  double error_min_frequency_hz_{0.0};
  std::string diagnostic_name_;
  std::string hardware_id_;

  typename MsgT::SharedPtr last_sensor_msg_;
  bool has_last_message_time_{false};
  double last_message_time_{0.0};
  bool has_last_frequency_message_time_{false};
  double last_frequency_message_time_{0.0};
  std::deque<double> frequency_timestamps_;

  typename rclcpp::Subscription<MsgT>::SharedPtr subscription_;
  typename rclcpp::Subscription<MsgT>::SharedPtr frequency_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sura_diagnostics
