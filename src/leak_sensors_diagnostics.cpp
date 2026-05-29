#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"
#include "sura_msgs/msg/leak_sensor.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sura_msgs::msg::LeakSensor;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

struct LeakSensorState
{
  bool leak_detected{false};
  double last_update_seconds{0.0};
};

class LeakSensorsDiagnostics : public rclcpp::Node
{
public:
  LeakSensorsDiagnostics()
  : Node("leak_sensors_diagnostics")
  {
    declare_parameter<std::string>("leak_topic", "");
    declare_parameter<std::string>("hardware_id", "leak_sensors");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("stale_timeout", 3.0);
    declare_parameter<std::vector<std::string>>(
      "sensor_frames", std::vector<std::string>{});

    leak_topic_ = get_parameter("leak_topic").as_string();
    hardware_id_ = get_parameter("hardware_id").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();

    subscription_ = create_subscription<LeakSensor>(
      leak_topic_, 10, [this](const LeakSensor::SharedPtr msg) { on_leak_sensor(msg); });
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() { publish_diagnostics(); });
  }

private:
  void on_leak_sensor(const LeakSensor::SharedPtr msg)
  {
    auto frame = trim(msg->frame_id);
    if (frame.empty()) {
      frame = trim(msg->header.frame_id);
    }
    if (frame.empty()) {
      RCLCPP_WARN(get_logger(), "Received leak sensor message without frame_id");
      return;
    }

    states_[frame] = LeakSensorState{msg->leak_detected, now_seconds()};
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    const auto seconds = time_seconds(now);
    const auto expected_frames = sensor_frames();

    std::vector<std::string> leaking_frames;
    std::vector<std::string> ok_frames;
    std::vector<std::string> stale_frames;
    for (const auto & frame : expected_frames) {
      const auto state = states_.find(frame);
      if (state == states_.end() || seconds - state->second.last_update_seconds > stale_timeout_) {
        stale_frames.push_back(frame);
      } else if (state->second.leak_detected) {
        leaking_frames.push_back(frame);
      } else {
        ok_frames.push_back(frame);
      }
    }

    DiagnosticStatus status;
    status.name = "/Sensors/LeakSensors";
    status.hardware_id = hardware_id_;
    if (!leaking_frames.empty()) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Leak detected";
    } else if (!stale_frames.empty()) {
      status.level = DiagnosticStatus::STALE;
      status.message = "Leak sensor data is stale";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = "Leak sensors healthy";
    }

    status.values = {
      key_value("topic", leak_topic_),
      key_value("sensor_count", std::to_string(expected_frames.size())),
      key_value("ok_frames", join(ok_frames)),
      key_value("leaking_frames", join(leaking_frames)),
      key_value("stale_frames", join(stale_frames)),
      key_value("stale_timeout", fixed_string(stale_timeout_, 3)),
    };

    DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    array_msg.status.push_back(status);
    diagnostics_pub_->publish(array_msg);
  }

  std::vector<std::string> sensor_frames()
  {
    auto frames = get_parameter("sensor_frames").as_string_array();
    if (!frames.empty()) {
      return frames;
    }
    frames.reserve(states_.size());
    for (const auto & item : states_) {
      frames.push_back(item.first);
    }
    return frames;
  }

  static std::string trim(const std::string & value)
  {
    const auto first = value.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = value.find_last_not_of(" \t\n\r");
    return value.substr(first, last - first + 1);
  }

  static std::string join(const std::vector<std::string> & values)
  {
    std::string output;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        output += ",";
      }
      output += values[index];
    }
    return output;
  }

  double now_seconds() { return time_seconds(get_clock()->now()); }
  static double time_seconds(const rclcpp::Time & time)
  {
    return static_cast<double>(time.nanoseconds()) / 1e9;
  }

  std::string leak_topic_;
  std::string hardware_id_{"leak_sensors"};
  double publish_period_{1.0};
  double stale_timeout_{3.0};
  std::map<std::string, LeakSensorState> states_;
  rclcpp::Subscription<LeakSensor>::SharedPtr subscription_;
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
