#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"
#include "sura_msgs/msg/navigation_safety.hpp"
#include "sura_msgs/msg/navigator.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sura_msgs::msg::NavigationSafety;
using sura_msgs::msg::Navigator;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;
using sura_diagnostics::optional_fixed_string;

class NavigationLimitsDiagnostics : public rclcpp::Node
{
public:
  NavigationLimitsDiagnostics()
  : Node("navigation_limits_diagnostics")
  {
    declare_parameter<std::string>("navigator_topic", "");
    declare_parameter<std::string>("frequency_topic", "");
    declare_parameter<std::string>("limits_hardware_id", "navigation_limits");
    declare_parameter<std::string>("frequency_hardware_id", "navigation");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("stale_timeout", 3.0);
    declare_parameter<double>("warn_min_frequency_hz", 10.0);
    declare_parameter<double>("error_min_frequency_hz", 2.0);
    declare_parameter<double>("max_depth", 5.0);
    declare_parameter<double>("depth_warning_margin", 0.5);
    declare_parameter<double>("min_altitude", 0.5);
    declare_parameter<double>("altitude_warning_margin", 0.5);
    declare_parameter<double>("center_x", 0.0);
    declare_parameter<double>("center_y", 0.0);
    declare_parameter<double>("max_radius", 20.0);
    declare_parameter<double>("radius_warning_margin", 2.0);

    navigator_topic_ = get_parameter("navigator_topic").as_string();
    frequency_topic_ = get_parameter("frequency_topic").as_string();
    limits_hardware_id_ = get_parameter("limits_hardware_id").as_string();
    frequency_hardware_id_ = get_parameter("frequency_hardware_id").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();
    warn_min_frequency_hz_ = get_parameter("warn_min_frequency_hz").as_double();
    error_min_frequency_hz_ = get_parameter("error_min_frequency_hz").as_double();
    max_depth_ = get_parameter("max_depth").as_double();
    depth_warning_margin_ = get_parameter("depth_warning_margin").as_double();
    min_altitude_ = get_parameter("min_altitude").as_double();
    altitude_warning_margin_ = get_parameter("altitude_warning_margin").as_double();
    center_x_ = get_parameter("center_x").as_double();
    center_y_ = get_parameter("center_y").as_double();
    max_radius_ = get_parameter("max_radius").as_double();
    radius_warning_margin_ = get_parameter("radius_warning_margin").as_double();

    subscription_ = create_subscription<Navigator>(
      navigator_topic_, 10, [this](const Navigator::SharedPtr msg) { on_navigator(msg); });
    frequency_subscription_ = create_subscription<Navigator>(
      frequency_topic_, 10, [this](const Navigator::SharedPtr msg) { on_frequency_navigator(msg); });
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    safety_pub_ = create_publisher<NavigationSafety>("safety/navigation", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() { publish_diagnostics(); });
  }

private:
  void on_navigator(const Navigator::SharedPtr msg)
  {
    last_navigation_msg_ = msg;
    last_message_time_ = now_seconds();
    has_last_message_time_ = true;
  }

  void on_frequency_navigator(const Navigator::SharedPtr)
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
    const auto has_age = has_last_message_time_;
    const auto age_seconds = has_age ? seconds - last_message_time_ : 0.0;
    const auto has_frequency_age = has_last_frequency_message_time_;
    const auto frequency_age_seconds =
      has_frequency_age ? seconds - last_frequency_message_time_ : 0.0;

    DiagnosticStatus depth_status;
    DiagnosticStatus altitude_status;
    DiagnosticStatus area_status;
    if (!last_navigation_msg_ || !has_age || age_seconds > stale_timeout_) {
      depth_status = build_stale_status("/Navigation/DepthLimit", has_age, age_seconds);
      altitude_status = build_stale_status("/Navigation/AltitudeLimit", has_age, age_seconds);
      area_status = build_stale_status("/Navigation/AreaLimit", has_age, age_seconds);
    } else {
      depth_status = build_depth_status(*last_navigation_msg_, age_seconds);
      altitude_status = build_altitude_status(*last_navigation_msg_, age_seconds);
      area_status = build_area_status(*last_navigation_msg_, age_seconds);
    }

    const auto safety_msg = build_safety_msg(now, has_age, age_seconds, has_frequency_age, frequency_age_seconds);
    const auto frequency_status = build_frequency_status(safety_msg, has_frequency_age, frequency_age_seconds);

    DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    array_msg.status = {depth_status, altitude_status, area_status, frequency_status};
    safety_pub_->publish(safety_msg);
    diagnostics_pub_->publish(array_msg);
  }

  NavigationSafety build_safety_msg(
    const rclcpp::Time & now, bool has_age, double age_seconds,
    bool has_frequency_age, double frequency_age_seconds)
  {
    NavigationSafety msg;
    msg.header.stamp = now;

    if (!last_navigation_msg_ || !has_age || age_seconds > stale_timeout_) {
      msg.depth_state = NavigationSafety::STALE;
      msg.depth = nan();
      msg.depth_remaining_to_warning = nan();
      msg.depth_remaining_to_error = nan();
      msg.area_state = NavigationSafety::STALE;
      msg.area_distance = nan();
      msg.area_remaining_to_warning = nan();
      msg.area_remaining_to_error = nan();
    } else {
      const auto current_depth = static_cast<double>(last_navigation_msg_->position.position.z);
      const auto depth_warn_limit = max_depth_ - depth_warning_margin_;
      msg.depth_state = state_from_limits(current_depth, depth_warn_limit, max_depth_);
      msg.depth = current_depth;
      msg.depth_remaining_to_warning = depth_warn_limit - current_depth;
      msg.depth_remaining_to_error = max_depth_ - current_depth;

      const auto current_x = static_cast<double>(last_navigation_msg_->position.position.x);
      const auto current_y = static_cast<double>(last_navigation_msg_->position.position.y);
      const auto area_distance = std::hypot(current_x - center_x_, current_y - center_y_);
      const auto area_warn_limit = max_radius_ - radius_warning_margin_;
      msg.area_state = state_from_limits(area_distance, area_warn_limit, max_radius_);
      msg.area_distance = area_distance;
      msg.area_remaining_to_warning = area_warn_limit - area_distance;
      msg.area_remaining_to_error = max_radius_ - area_distance;
    }

    msg.depth_limit = max_depth_;
    msg.area_limit = max_radius_;

    const auto frequency = frequency_hz();
    msg.frequency_hz = frequency;
    msg.frequency_warn_limit = warn_min_frequency_hz_;
    msg.frequency_error_limit = error_min_frequency_hz_;
    msg.frequency_remaining_to_warning = frequency - warn_min_frequency_hz_;
    msg.frequency_remaining_to_error = frequency - error_min_frequency_hz_;
    if (!has_frequency_age || frequency_age_seconds > stale_timeout_) {
      msg.frequency_state = NavigationSafety::STALE;
    } else if (frequency < error_min_frequency_hz_) {
      msg.frequency_state = NavigationSafety::ERROR;
    } else if (frequency < warn_min_frequency_hz_) {
      msg.frequency_state = NavigationSafety::WARN;
    } else {
      msg.frequency_state = NavigationSafety::OK;
    }
    return msg;
  }

  DiagnosticStatus build_stale_status(const std::string & name, bool has_age, double age_seconds)
  {
    DiagnosticStatus status;
    status.name = name;
    status.hardware_id = limits_hardware_id_;
    status.level = DiagnosticStatus::STALE;
    status.message = "Navigation limits input is stale";
    status.values = base_values(has_age, age_seconds);
    return status;
  }

  DiagnosticStatus build_frequency_status(
    const NavigationSafety & safety_msg, bool has_age, double age_seconds)
  {
    DiagnosticStatus status;
    status.name = "/Navigation/Frequency";
    status.hardware_id = frequency_hardware_id_;
    if (safety_msg.frequency_state == NavigationSafety::STALE) {
      status.level = DiagnosticStatus::STALE;
      status.message = "Navigator topic is stale";
    } else if (safety_msg.frequency_state == NavigationSafety::ERROR) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Navigator frequency is critically low";
    } else if (safety_msg.frequency_state == NavigationSafety::WARN) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Navigator frequency is low";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = "Navigator frequency healthy";
    }
    status.values = {
      key_value("topic", frequency_topic_),
      key_value("frequency_hz", fixed_string(safety_msg.frequency_hz, 3)),
      key_value("sample_count", std::to_string(frequency_timestamps_.size())),
      key_value("window_seconds", fixed_string(publish_period_, 3)),
      key_value("warn_min_frequency_hz", fixed_string(warn_min_frequency_hz_, 3)),
      key_value("error_min_frequency_hz", fixed_string(error_min_frequency_hz_, 3)),
      key_value("age_seconds", optional_fixed_string(has_age, age_seconds, 3)),
    };
    return status;
  }

  DiagnosticStatus build_depth_status(const Navigator & msg, double age_seconds)
  {
    const auto current_depth = static_cast<double>(msg.position.position.z);
    const auto remaining_depth = max_depth_ - current_depth;
    DiagnosticStatus status;
    status.name = "/Navigation/DepthLimit";
    status.hardware_id = limits_hardware_id_;
    if (remaining_depth < 0.0) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Maximum depth exceeded";
    } else if (remaining_depth <= depth_warning_margin_) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Approaching maximum depth";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = "Depth inside limits";
    }
    status.values = base_values(true, age_seconds);
    append_values(status.values, {
      key_value("current_depth", fixed_string(current_depth, 3)),
      key_value("max_depth", fixed_string(max_depth_, 3)),
      key_value("remaining_depth_margin", fixed_string(remaining_depth, 3)),
      key_value("depth_warning_margin", fixed_string(depth_warning_margin_, 3)),
    });
    return status;
  }

  DiagnosticStatus build_altitude_status(const Navigator & msg, double age_seconds)
  {
    const auto current_altitude = static_cast<double>(msg.altitude);
    const auto remaining_altitude = current_altitude - min_altitude_;
    DiagnosticStatus status;
    status.name = "/Navigation/AltitudeLimit";
    status.hardware_id = limits_hardware_id_;
    if (!std::isfinite(current_altitude) || current_altitude < 0.0) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Altitude data is invalid";
    } else if (remaining_altitude < 0.0) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Minimum altitude exceeded";
    } else if (remaining_altitude <= altitude_warning_margin_) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Approaching minimum altitude";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = "Altitude inside limits";
    }
    status.values = base_values(true, age_seconds);
    append_values(status.values, {
      key_value("current_altitude", fixed_string(current_altitude, 3)),
      key_value("min_altitude", fixed_string(min_altitude_, 3)),
      key_value("remaining_altitude_margin", fixed_string(remaining_altitude, 3)),
      key_value("altitude_warning_margin", fixed_string(altitude_warning_margin_, 3)),
    });
    return status;
  }

  DiagnosticStatus build_area_status(const Navigator & msg, double age_seconds)
  {
    const auto current_x = static_cast<double>(msg.position.position.x);
    const auto current_y = static_cast<double>(msg.position.position.y);
    const auto distance_xy = std::hypot(current_x - center_x_, current_y - center_y_);
    const auto remaining_radius = max_radius_ - distance_xy;
    DiagnosticStatus status;
    status.name = "/Navigation/AreaLimit";
    status.hardware_id = limits_hardware_id_;
    if (remaining_radius < 0.0) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Maximum navigation radius exceeded";
    } else if (remaining_radius <= radius_warning_margin_) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Approaching maximum navigation radius";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message = "Navigation area inside limits";
    }
    status.values = base_values(true, age_seconds);
    append_values(status.values, {
      key_value("current_x", fixed_string(current_x, 3)),
      key_value("current_y", fixed_string(current_y, 3)),
      key_value("center_x", fixed_string(center_x_, 3)),
      key_value("center_y", fixed_string(center_y_, 3)),
      key_value("distance_xy", fixed_string(distance_xy, 3)),
      key_value("max_radius", fixed_string(max_radius_, 3)),
      key_value("remaining_radius_margin", fixed_string(remaining_radius, 3)),
      key_value("radius_warning_margin", fixed_string(radius_warning_margin_, 3)),
    });
    return status;
  }

  std::vector<KeyValue> base_values(bool has_age, double age_seconds) const
  {
    return {
      key_value("topic", navigator_topic_),
      key_value("age_seconds", optional_fixed_string(has_age, age_seconds, 3)),
      key_value("stale_timeout", fixed_string(stale_timeout_, 3)),
    };
  }

  static void append_values(std::vector<KeyValue> & target, const std::vector<KeyValue> & values)
  {
    target.insert(target.end(), values.begin(), values.end());
  }

  void drop_old_frequency_timestamps(double seconds)
  {
    while (!frequency_timestamps_.empty() &&
      seconds - frequency_timestamps_.front() > publish_period_)
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

  unsigned char state_from_limits(double value, double warning_limit, double error_limit) const
  {
    if (value > error_limit) {
      return NavigationSafety::ERROR;
    }
    if (value >= warning_limit) {
      return NavigationSafety::WARN;
    }
    return NavigationSafety::OK;
  }

  static double nan() { return std::numeric_limits<double>::quiet_NaN(); }
  double now_seconds() { return time_seconds(get_clock()->now()); }
  static double time_seconds(const rclcpp::Time & time)
  {
    return static_cast<double>(time.nanoseconds()) / 1e9;
  }

  std::string navigator_topic_;
  std::string frequency_topic_;
  std::string limits_hardware_id_{"navigation_limits"};
  std::string frequency_hardware_id_{"navigation"};
  double publish_period_{1.0};
  double stale_timeout_{3.0};
  double warn_min_frequency_hz_{10.0};
  double error_min_frequency_hz_{2.0};
  double max_depth_{5.0};
  double depth_warning_margin_{0.5};
  double min_altitude_{0.5};
  double altitude_warning_margin_{0.5};
  double center_x_{0.0};
  double center_y_{0.0};
  double max_radius_{20.0};
  double radius_warning_margin_{2.0};
  Navigator::SharedPtr last_navigation_msg_;
  bool has_last_message_time_{false};
  double last_message_time_{0.0};
  std::deque<double> frequency_timestamps_;
  bool has_last_frequency_message_time_{false};
  double last_frequency_message_time_{0.0};
  rclcpp::Subscription<Navigator>::SharedPtr subscription_;
  rclcpp::Subscription<Navigator>::SharedPtr frequency_subscription_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<NavigationSafety>::SharedPtr safety_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationLimitsDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
