#include <algorithm>
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
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sura_msgs::msg::NavigationSafety;
using sura_msgs::msg::Navigator;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;
using sura_diagnostics::optional_fixed_string;
using sura_diagnostics::prefixed_diagnostic_name;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class NavigationLimitsDiagnostics : public rclcpp::Node
{
public:
  NavigationLimitsDiagnostics()
  : Node("navigation_limits_diagnostics")
  {
    declare_parameter<std::string>("navigator_topic", "");
    declare_parameter<std::string>("frequency_topic", "");
    declare_parameter<std::string>("marker_topic", "navigation_limits/markers");
    declare_parameter<std::string>("marker_frame_id", "world_ned");
    declare_parameter<std::string>("limits_hardware_id", "navigation_limits");
    declare_parameter<std::string>("frequency_hardware_id", "navigation");
    declare_parameter<std::string>("robot_namespace", "");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("frequency_window", 5.0);
    declare_parameter<double>("frequency_stale_timeout", 5.0);
    declare_parameter<double>("stale_timeout", 3.0);
    declare_parameter<double>("warn_min_frequency_hz", 10.0);
    declare_parameter<double>("error_min_frequency_hz", 0.5);
    declare_parameter<double>("max_depth", 5.0);
    declare_parameter<double>("depth_warning_margin", 0.5);
    declare_parameter<double>("min_altitude", 0.5);
    declare_parameter<double>("altitude_warning_margin", 0.5);
    declare_parameter<double>("center_x", 0.0);
    declare_parameter<double>("center_y", 0.0);
    declare_parameter<double>("max_radius", 20.0);
    declare_parameter<double>("radius_warning_margin", 2.0);
    const auto legacy_max_depth = get_parameter("max_depth").as_double();
    const auto legacy_depth_warning_margin = get_parameter("depth_warning_margin").as_double();
    const auto legacy_min_altitude = get_parameter("min_altitude").as_double();
    const auto legacy_altitude_warning_margin = get_parameter("altitude_warning_margin").as_double();
    const auto legacy_center_x = get_parameter("center_x").as_double();
    const auto legacy_center_y = get_parameter("center_y").as_double();
    const auto legacy_max_radius = get_parameter("max_radius").as_double();
    const auto legacy_radius_warning_margin = get_parameter("radius_warning_margin").as_double();

    declare_parameter<double>("depth.max", legacy_max_depth);
    declare_parameter<double>("depth.warning_margin", legacy_depth_warning_margin);
    declare_parameter<double>("altitude.min", legacy_min_altitude);
    declare_parameter<double>("altitude.warning_margin", legacy_altitude_warning_margin);
    declare_parameter<std::string>("area.shape", "circle");
    declare_parameter<double>("area.center.x", legacy_center_x);
    declare_parameter<double>("area.center.y", legacy_center_y);
    declare_parameter<double>("area.circle.radius", legacy_max_radius);
    declare_parameter<double>("area.rectangle.width", legacy_max_radius * 2.0);
    declare_parameter<double>("area.rectangle.height", legacy_max_radius * 2.0);
    declare_parameter<double>("area.warning_margin", legacy_radius_warning_margin);

    navigator_topic_ = get_parameter("navigator_topic").as_string();
    frequency_topic_ = get_parameter("frequency_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    marker_frame_id_ = get_parameter("marker_frame_id").as_string();
    limits_hardware_id_ = get_parameter("limits_hardware_id").as_string();
    frequency_hardware_id_ = get_parameter("frequency_hardware_id").as_string();
    robot_namespace_ = get_parameter("robot_namespace").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    frequency_window_ = get_parameter("frequency_window").as_double();
    frequency_stale_timeout_ = get_parameter("frequency_stale_timeout").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();
    warn_min_frequency_hz_ = get_parameter("warn_min_frequency_hz").as_double();
    error_min_frequency_hz_ = get_parameter("error_min_frequency_hz").as_double();
    max_depth_ = get_parameter("depth.max").as_double();
    depth_warning_margin_ = get_parameter("depth.warning_margin").as_double();
    min_altitude_ = get_parameter("altitude.min").as_double();
    altitude_warning_margin_ = get_parameter("altitude.warning_margin").as_double();
    area_shape_ = get_parameter("area.shape").as_string();
    center_x_ = get_parameter("area.center.x").as_double();
    center_y_ = get_parameter("area.center.y").as_double();
    max_radius_ = get_parameter("area.circle.radius").as_double();
    rectangle_width_ = get_parameter("area.rectangle.width").as_double();
    rectangle_height_ = get_parameter("area.rectangle.height").as_double();
    radius_warning_margin_ = get_parameter("area.warning_margin").as_double();

    subscription_ = create_subscription<Navigator>(
      navigator_topic_, 10, [this](const Navigator::SharedPtr msg) { on_navigator(msg); });
    frequency_subscription_ = create_subscription<Navigator>(
      frequency_topic_, 10, [this](const Navigator::SharedPtr msg) { on_frequency_navigator(msg); });
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    safety_pub_ = create_publisher<NavigationSafety>("safety/navigation", 10);
    markers_pub_ = create_publisher<MarkerArray>(marker_topic_, 10);
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
    drop_old_frequency_timestamps(seconds, frequency_window_);
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    const auto seconds = time_seconds(now);
    drop_old_frequency_timestamps(seconds, frequency_window_);
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
    markers_pub_->publish(build_marker_array(now));
    diagnostics_pub_->publish(array_msg);
  }

  MarkerArray build_marker_array(const rclcpp::Time & stamp) const
  {
    MarkerArray markers;
    const auto warning_depth = max_depth_ - depth_warning_margin_;

    if (area_is_rectangle()) {
      const auto half_width = rectangle_width_ * 0.5;
      const auto half_height = rectangle_height_ * 0.5;
      const auto warning_half_width = half_width - radius_warning_margin_;
      const auto warning_half_height = half_height - radius_warning_margin_;

      markers.markers.push_back(build_rectangle_wall_marker(
        stamp, "navigation_area_walls", 10, half_width, half_height, 0.0, max_depth_,
        1.0F, 0.1F, 0.1F, 0.18F));
      markers.markers.push_back(build_rectangle_marker(
        stamp, "navigation_area", 0, half_width, half_height, 0.0, 1.0F, 0.1F, 0.1F, 0.9F));
      markers.markers.push_back(build_rectangle_plane_marker(
        stamp, "navigation_depth_planes", 12, half_width, half_height, max_depth_,
        1.0F, 0.1F, 0.1F, 0.18F));
      markers.markers.push_back(build_rectangle_marker(
        stamp, "navigation_depth", 2, half_width, half_height, max_depth_,
        1.0F, 0.1F, 0.1F, 0.9F));

      if (warning_half_width > 0.0 && warning_half_height > 0.0 && warning_depth > 0.0) {
        markers.markers.push_back(build_rectangle_wall_marker(
          stamp, "navigation_area_walls", 11, warning_half_width, warning_half_height, 0.0,
          warning_depth, 1.0F, 0.75F, 0.0F, 0.12F));
        markers.markers.push_back(build_rectangle_marker(
          stamp, "navigation_area", 1, warning_half_width, warning_half_height, 0.0,
          1.0F, 0.75F, 0.0F, 0.9F));
        markers.markers.push_back(build_rectangle_plane_marker(
          stamp, "navigation_depth_planes", 13, warning_half_width, warning_half_height,
          warning_depth, 1.0F, 0.75F, 0.0F, 0.12F));
        markers.markers.push_back(build_rectangle_marker(
          stamp, "navigation_depth", 3, warning_half_width, warning_half_height, warning_depth,
          1.0F, 0.75F, 0.0F, 0.9F));
      }
      return markers;
    }

    markers.markers.push_back(build_cylinder_wall_marker(
      stamp, "navigation_area_walls", 10, max_radius_, 0.0, max_depth_, 1.0F, 0.1F, 0.1F, 0.18F));
    markers.markers.push_back(build_circle_marker(
      stamp, "navigation_area", 0, max_radius_, 0.0, 1.0F, 0.1F, 0.1F, 0.9F));

    const auto warning_radius = max_radius_ - radius_warning_margin_;
    if (warning_radius > 0.0 && warning_depth > 0.0) {
      markers.markers.push_back(build_cylinder_wall_marker(
        stamp, "navigation_area_walls", 11, warning_radius, 0.0, warning_depth, 1.0F, 0.75F, 0.0F, 0.12F));
      markers.markers.push_back(build_circle_marker(
        stamp, "navigation_area", 1, warning_radius, 0.0, 1.0F, 0.75F, 0.0F, 0.9F));
    }

    markers.markers.push_back(build_depth_plane_marker(
      stamp, "navigation_depth_planes", 12, max_radius_, max_depth_, 1.0F, 0.1F, 0.1F, 0.18F));
    markers.markers.push_back(build_circle_marker(
      stamp, "navigation_depth", 2, max_radius_, max_depth_, 1.0F, 0.1F, 0.1F, 0.9F));

    if (warning_radius > 0.0 && warning_depth > 0.0) {
      markers.markers.push_back(build_depth_plane_marker(
        stamp, "navigation_depth_planes", 13, warning_radius, warning_depth, 1.0F, 0.75F, 0.0F, 0.12F));
      markers.markers.push_back(build_circle_marker(
        stamp, "navigation_depth", 3, warning_radius, warning_depth, 1.0F, 0.75F, 0.0F, 0.9F));
    }
    return markers;
  }

  Marker build_circle_marker(
    const rclcpp::Time & stamp, const std::string & ns, int id, double radius, double z,
    float red, float green, float blue, float alpha) const
  {
    Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    constexpr int kSegments = 96;
    const auto two_pi = 2.0 * std::acos(-1.0);
    marker.points.reserve(kSegments + 1);
    for (int i = 0; i <= kSegments; ++i) {
      const auto angle = two_pi * static_cast<double>(i) / static_cast<double>(kSegments);
      geometry_msgs::msg::Point point;
      point.x = center_x_ + radius * std::cos(angle);
      point.y = center_y_ + radius * std::sin(angle);
      point.z = z;
      marker.points.push_back(point);
    }
    return marker;
  }

  Marker build_rectangle_marker(
    const rclcpp::Time & stamp, const std::string & ns, int id, double half_width,
    double half_height, double z, float red, float green, float blue, float alpha) const
  {
    Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::LINE_STRIP;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    marker.points.push_back(point_at_offset(-half_width, -half_height, z));
    marker.points.push_back(point_at_offset(half_width, -half_height, z));
    marker.points.push_back(point_at_offset(half_width, half_height, z));
    marker.points.push_back(point_at_offset(-half_width, half_height, z));
    marker.points.push_back(point_at_offset(-half_width, -half_height, z));
    return marker;
  }

  Marker build_cylinder_wall_marker(
    const rclcpp::Time & stamp, const std::string & ns, int id, double radius,
    double min_z, double max_z, float red, float green, float blue, float alpha) const
  {
    Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::TRIANGLE_LIST;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    constexpr int kSegments = 96;
    const auto two_pi = 2.0 * std::acos(-1.0);
    marker.points.reserve(kSegments * 6);
    for (int i = 0; i < kSegments; ++i) {
      const auto angle_a = two_pi * static_cast<double>(i) / static_cast<double>(kSegments);
      const auto angle_b = two_pi * static_cast<double>(i + 1) / static_cast<double>(kSegments);

      const auto bottom_a = point_on_circle(radius, angle_a, min_z);
      const auto bottom_b = point_on_circle(radius, angle_b, min_z);
      const auto top_b = point_on_circle(radius, angle_b, max_z);
      const auto top_a = point_on_circle(radius, angle_a, max_z);

      marker.points.push_back(bottom_a);
      marker.points.push_back(bottom_b);
      marker.points.push_back(top_b);
      marker.points.push_back(bottom_a);
      marker.points.push_back(top_b);
      marker.points.push_back(top_a);
    }
    return marker;
  }

  Marker build_rectangle_wall_marker(
    const rclcpp::Time & stamp, const std::string & ns, int id, double half_width,
    double half_height, double min_z, double max_z, float red, float green, float blue,
    float alpha) const
  {
    Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::TRIANGLE_LIST;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    const std::vector<geometry_msgs::msg::Point> bottom = {
      point_at_offset(-half_width, -half_height, min_z),
      point_at_offset(half_width, -half_height, min_z),
      point_at_offset(half_width, half_height, min_z),
      point_at_offset(-half_width, half_height, min_z),
    };
    const std::vector<geometry_msgs::msg::Point> top = {
      point_at_offset(-half_width, -half_height, max_z),
      point_at_offset(half_width, -half_height, max_z),
      point_at_offset(half_width, half_height, max_z),
      point_at_offset(-half_width, half_height, max_z),
    };

    marker.points.reserve(24);
    for (std::size_t i = 0; i < bottom.size(); ++i) {
      const auto next = (i + 1) % bottom.size();
      marker.points.push_back(bottom[i]);
      marker.points.push_back(bottom[next]);
      marker.points.push_back(top[next]);
      marker.points.push_back(bottom[i]);
      marker.points.push_back(top[next]);
      marker.points.push_back(top[i]);
    }
    return marker;
  }

  Marker build_depth_plane_marker(
    const rclcpp::Time & stamp, const std::string & ns, int id, double radius, double z,
    float red, float green, float blue, float alpha) const
  {
    Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::TRIANGLE_LIST;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    constexpr int kSegments = 96;
    const auto two_pi = 2.0 * std::acos(-1.0);
    marker.points.reserve(kSegments * 3);
    for (int i = 0; i < kSegments; ++i) {
      const auto angle_a = two_pi * static_cast<double>(i) / static_cast<double>(kSegments);
      const auto angle_b = two_pi * static_cast<double>(i + 1) / static_cast<double>(kSegments);
      geometry_msgs::msg::Point center;
      center.x = center_x_;
      center.y = center_y_;
      center.z = z;
      marker.points.push_back(center);
      marker.points.push_back(point_on_circle(radius, angle_a, z));
      marker.points.push_back(point_on_circle(radius, angle_b, z));
    }
    return marker;
  }

  Marker build_rectangle_plane_marker(
    const rclcpp::Time & stamp, const std::string & ns, int id, double half_width,
    double half_height, double z, float red, float green, float blue, float alpha) const
  {
    Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = Marker::TRIANGLE_LIST;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    const auto a = point_at_offset(-half_width, -half_height, z);
    const auto b = point_at_offset(half_width, -half_height, z);
    const auto c = point_at_offset(half_width, half_height, z);
    const auto d = point_at_offset(-half_width, half_height, z);

    marker.points = {a, b, c, a, c, d};
    return marker;
  }

  geometry_msgs::msg::Point point_on_circle(double radius, double angle, double z) const
  {
    geometry_msgs::msg::Point point;
    point.x = center_x_ + radius * std::cos(angle);
    point.y = center_y_ + radius * std::sin(angle);
    point.z = z;
    return point;
  }

  geometry_msgs::msg::Point point_at_offset(double x_offset, double y_offset, double z) const
  {
    geometry_msgs::msg::Point point;
    point.x = center_x_ + x_offset;
    point.y = center_y_ + y_offset;
    point.z = z;
    return point;
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
      const auto area = evaluate_area(current_x, current_y);
      msg.area_state = area.state;
      msg.area_distance = area.distance;
      msg.area_remaining_to_warning = area.remaining_to_warning;
      msg.area_remaining_to_error = area.remaining_to_error;
    }

    msg.depth_limit = max_depth_;
    msg.area_limit = area_is_rectangle() ?
      std::min(rectangle_width_, rectangle_height_) * 0.5 : max_radius_;

    const auto frequency = frequency_hz();
    msg.frequency_hz = frequency;
    msg.frequency_warn_limit = warn_min_frequency_hz_;
    msg.frequency_error_limit = error_min_frequency_hz_;
    msg.frequency_remaining_to_warning = frequency - warn_min_frequency_hz_;
    msg.frequency_remaining_to_error = frequency - error_min_frequency_hz_;
    if (!has_frequency_age || frequency_age_seconds > frequency_stale_timeout_) {
      msg.frequency_state = NavigationSafety::STALE;
    } else if (frequency_timestamps_.size() < 2) {
      msg.frequency_state = NavigationSafety::WARN;
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
    status.name = prefixed_diagnostic_name(robot_namespace_, name);
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
    status.name = prefixed_diagnostic_name(robot_namespace_, "/Navigation/Frequency");
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
      key_value("window_seconds", fixed_string(frequency_window_, 3)),
      key_value("warn_min_frequency_hz", fixed_string(warn_min_frequency_hz_, 3)),
      key_value("error_min_frequency_hz", fixed_string(error_min_frequency_hz_, 3)),
      key_value("frequency_stale_timeout", fixed_string(frequency_stale_timeout_, 3)),
      key_value("age_seconds", optional_fixed_string(has_age, age_seconds, 3)),
    };
    return status;
  }

  DiagnosticStatus build_depth_status(const Navigator & msg, double age_seconds)
  {
    const auto current_depth = static_cast<double>(msg.position.position.z);
    const auto remaining_depth = max_depth_ - current_depth;
    DiagnosticStatus status;
    status.name = prefixed_diagnostic_name(robot_namespace_, "/Navigation/DepthLimit");
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
    status.name = prefixed_diagnostic_name(robot_namespace_, "/Navigation/AltitudeLimit");
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
    const auto area = evaluate_area(current_x, current_y);
    DiagnosticStatus status;
    status.name = prefixed_diagnostic_name(robot_namespace_, "/Navigation/AreaLimit");
    status.hardware_id = limits_hardware_id_;
    if (area.state == NavigationSafety::ERROR) {
      status.level = DiagnosticStatus::ERROR;
      status.message = area_is_rectangle() ?
        "Navigation rectangle exceeded" : "Maximum navigation radius exceeded";
    } else if (area.state == NavigationSafety::WARN) {
      status.level = DiagnosticStatus::WARN;
      status.message = area_is_rectangle() ?
        "Approaching navigation rectangle boundary" : "Approaching maximum navigation radius";
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
      key_value("shape", area_shape_),
      key_value("distance_xy", fixed_string(area.distance, 3)),
      key_value("remaining_area_margin", fixed_string(area.remaining_to_error, 3)),
      key_value("area_warning_margin", fixed_string(radius_warning_margin_, 3)),
    });
    if (area_is_rectangle()) {
      append_values(status.values, {
        key_value("width", fixed_string(rectangle_width_, 3)),
        key_value("height", fixed_string(rectangle_height_, 3)),
      });
    } else {
      append_values(status.values, {
        key_value("radius", fixed_string(max_radius_, 3)),
      });
    }
    return status;
  }

  struct AreaEvaluation
  {
    unsigned char state{NavigationSafety::OK};
    double distance{0.0};
    double remaining_to_warning{0.0};
    double remaining_to_error{0.0};
  };

  AreaEvaluation evaluate_area(double current_x, double current_y) const
  {
    const auto dx = current_x - center_x_;
    const auto dy = current_y - center_y_;
    if (area_is_rectangle()) {
      const auto half_width = rectangle_width_ * 0.5;
      const auto half_height = rectangle_height_ * 0.5;
      const auto remaining_x = half_width - std::abs(dx);
      const auto remaining_y = half_height - std::abs(dy);
      const auto remaining = std::min(remaining_x, remaining_y);
      AreaEvaluation evaluation;
      evaluation.distance = std::hypot(dx, dy);
      evaluation.remaining_to_warning = remaining - radius_warning_margin_;
      evaluation.remaining_to_error = remaining;
      if (remaining < 0.0) {
        evaluation.state = NavigationSafety::ERROR;
      } else if (remaining <= radius_warning_margin_) {
        evaluation.state = NavigationSafety::WARN;
      }
      return evaluation;
    }

    const auto distance_xy = std::hypot(dx, dy);
    const auto remaining_radius = max_radius_ - distance_xy;
    AreaEvaluation evaluation;
    evaluation.distance = distance_xy;
    evaluation.remaining_to_warning = remaining_radius - radius_warning_margin_;
    evaluation.remaining_to_error = remaining_radius;
    if (remaining_radius < 0.0) {
      evaluation.state = NavigationSafety::ERROR;
    } else if (remaining_radius <= radius_warning_margin_) {
      evaluation.state = NavigationSafety::WARN;
    }
    return evaluation;
  }

  bool area_is_rectangle() const
  {
    return area_shape_ == "rectangle" || area_shape_ == "rect";
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

  void drop_old_frequency_timestamps(double seconds, double window)
  {
    while (!frequency_timestamps_.empty() &&
      seconds - frequency_timestamps_.front() > window)
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
  std::string marker_topic_;
  std::string marker_frame_id_{"world_ned"};
  std::string limits_hardware_id_{"navigation_limits"};
  std::string frequency_hardware_id_{"navigation"};
  std::string robot_namespace_;
  double publish_period_{1.0};
  double frequency_window_{5.0};
  double frequency_stale_timeout_{5.0};
  double stale_timeout_{3.0};
  double warn_min_frequency_hz_{10.0};
  double error_min_frequency_hz_{0.5};
  double max_depth_{5.0};
  double depth_warning_margin_{0.5};
  double min_altitude_{0.5};
  double altitude_warning_margin_{0.5};
  std::string area_shape_{"circle"};
  double center_x_{0.0};
  double center_y_{0.0};
  double max_radius_{20.0};
  double rectangle_width_{40.0};
  double rectangle_height_{40.0};
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
  rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationLimitsDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
