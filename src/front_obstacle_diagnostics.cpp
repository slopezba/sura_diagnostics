#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sensor_msgs::msg::Image;
using sura_diagnostics::bool_string;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;
using sura_diagnostics::optional_fixed_string;

class FrontObstacleDiagnostics : public rclcpp::Node
{
public:
  FrontObstacleDiagnostics()
  : Node("front_obstacle_diagnostics")
  {
    declare_parameter<std::string>("depth_image_topic", "");
    declare_parameter<std::string>("sensor_hardware_id", "front_depth_camera");
    declare_parameter<std::string>("navigation_hardware_id", "front_obstacle");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("stale_timeout", 3.0);
    declare_parameter<double>("frequency_window", 5.0);
    declare_parameter<double>("warn_min_frequency_hz", 5.0);
    declare_parameter<double>("warning_distance_m", 1.0);
    declare_parameter<double>("error_distance_m", 0.5);
    declare_parameter<double>("min_valid_depth_m", 0.05);
    declare_parameter<double>("max_valid_depth_m", 10.0);
    declare_parameter<double>("roi_center_fraction_width", 0.6);
    declare_parameter<double>("roi_center_fraction_height", 0.6);
    declare_parameter<double>("min_valid_pixel_ratio", 0.01);

    depth_image_topic_ = get_parameter("depth_image_topic").as_string();
    sensor_hardware_id_ = get_parameter("sensor_hardware_id").as_string();
    navigation_hardware_id_ = get_parameter("navigation_hardware_id").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();
    frequency_window_ = get_parameter("frequency_window").as_double();
    warn_min_frequency_hz_ = get_parameter("warn_min_frequency_hz").as_double();

    subscription_ = create_subscription<Image>(
      depth_image_topic_, 10, [this](const Image::SharedPtr msg) { on_depth_image(msg); });
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() { publish_diagnostics(); });
  }

private:
  struct DepthStats
  {
    bool supported_encoding{false};
    double min_depth_m{std::numeric_limits<double>::quiet_NaN()};
    std::size_t valid_pixels{0};
    std::size_t roi_pixels{0};
  };

  struct RoiBounds
  {
    uint32_t x_begin{0};
    uint32_t x_end{0};
    uint32_t y_begin{0};
    uint32_t y_end{0};
  };

  void on_depth_image(const Image::SharedPtr msg)
  {
    last_image_ = msg;
    const auto seconds = now_seconds();
    last_message_time_ = seconds;
    has_last_message_time_ = true;
    frequency_timestamps_.push_back(seconds);
    drop_old_frequency_timestamps(seconds);
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    const auto seconds = time_seconds(now);
    drop_old_frequency_timestamps(seconds);

    DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    array_msg.status.push_back(build_sensor_status(seconds));
    array_msg.status.push_back(build_navigation_status(seconds));
    diagnostics_pub_->publish(array_msg);
  }

  DiagnosticStatus build_sensor_status(double seconds) const
  {
    const auto age = age_seconds(seconds);
    const auto frequency = frequency_hz();

    DiagnosticStatus status;
    status.name = "/Sensors/FrontDepthCamera";
    status.hardware_id = sensor_hardware_id_;
    status.level = DiagnosticStatus::OK;
    status.message = "Front depth camera data is being received";

    if (!age.first || age.second > stale_timeout_) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Front depth camera data is not being received";
    } else if (frequency < warn_min_frequency_hz_) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Front depth camera frequency is low";
    }

    status.values = base_values(age, frequency);
    return status;
  }

  DiagnosticStatus build_navigation_status(double seconds) const
  {
    const auto age = age_seconds(seconds);
    const auto frequency = frequency_hz();

    DiagnosticStatus status;
    status.name = "/Navigation/FrontObstacle";
    status.hardware_id = navigation_hardware_id_;

    std::vector<KeyValue> validation_values;
    if (!last_image_ || !age.first || age.second > stale_timeout_) {
      status.level = DiagnosticStatus::STALE;
      status.message = "Front obstacle distance is unknown";
    } else {
      validation_values = evaluate_obstacle(*last_image_, status);
    }

    status.values = base_values(age, frequency);
    status.values.insert(status.values.end(), validation_values.begin(), validation_values.end());
    return status;
  }

  std::vector<KeyValue> evaluate_obstacle(const Image & msg, DiagnosticStatus & status) const
  {
    const auto warning_distance = get_parameter("warning_distance_m").as_double();
    const auto error_distance = get_parameter("error_distance_m").as_double();
    const auto min_valid_depth = get_parameter("min_valid_depth_m").as_double();
    const auto max_valid_depth = get_parameter("max_valid_depth_m").as_double();
    const auto min_valid_pixel_ratio = get_parameter("min_valid_pixel_ratio").as_double();

    const auto stats = depth_stats(msg, min_valid_depth, max_valid_depth);
    const auto valid_pixel_ratio = stats.roi_pixels > 0 ?
      static_cast<double>(stats.valid_pixels) / static_cast<double>(stats.roi_pixels) :
      0.0;
    const auto has_enough_depth = stats.valid_pixels > 0 && valid_pixel_ratio >= min_valid_pixel_ratio;

    status.level = DiagnosticStatus::OK;
    status.message = "Front obstacle distance healthy";
    if (!stats.supported_encoding) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Depth image encoding is not supported";
    } else if (!has_enough_depth) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Front obstacle distance has too few valid depth pixels";
    } else if (stats.min_depth_m <= error_distance) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Front obstacle critically close";
    } else if (stats.min_depth_m <= warning_distance) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Front obstacle close";
    }

    return {
      key_value("encoding", msg.encoding),
      key_value("width", std::to_string(msg.width)),
      key_value("height", std::to_string(msg.height)),
      key_value("roi_pixels", std::to_string(stats.roi_pixels)),
      key_value("valid_pixels", std::to_string(stats.valid_pixels)),
      key_value("valid_pixel_ratio", fixed_string(valid_pixel_ratio, 4)),
      key_value("min_depth_m", fixed_string(stats.min_depth_m, 4)),
      key_value("warning_distance_m", fixed_string(warning_distance, 4)),
      key_value("error_distance_m", fixed_string(error_distance, 4)),
      key_value("min_valid_depth_m", fixed_string(min_valid_depth, 4)),
      key_value("max_valid_depth_m", fixed_string(max_valid_depth, 4)),
      key_value("supported_encoding", bool_string(stats.supported_encoding)),
    };
  }

  std::vector<KeyValue> base_values(
    const std::pair<bool, double> & age, double frequency) const
  {
    return {
      key_value("depth_image_topic", depth_image_topic_),
      key_value("frequency_hz", fixed_string(frequency, 3)),
      key_value("sample_count", std::to_string(frequency_timestamps_.size())),
      key_value("frequency_window", fixed_string(frequency_window_, 3)),
      key_value("warn_min_frequency_hz", fixed_string(warn_min_frequency_hz_, 3)),
      key_value("age_seconds", optional_fixed_string(age.first, age.second, 3)),
      key_value("stale_timeout", fixed_string(stale_timeout_, 3)),
    };
  }

  DepthStats depth_stats(const Image & msg, double min_valid_depth, double max_valid_depth) const
  {
    DepthStats stats;
    stats.supported_encoding = is_supported_encoding(msg.encoding);
    if (!stats.supported_encoding || msg.width == 0 || msg.height == 0) {
      return stats;
    }

    const auto bytes_per_pixel = msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ? 4U : 2U;
    if (msg.step < msg.width * bytes_per_pixel) {
      return stats;
    }

    const auto roi = roi_bounds(msg.width, msg.height);
    stats.roi_pixels = static_cast<std::size_t>(roi.x_end - roi.x_begin) *
      static_cast<std::size_t>(roi.y_end - roi.y_begin);
    stats.min_depth_m = std::numeric_limits<double>::infinity();

    for (uint32_t y = roi.y_begin; y < roi.y_end; ++y) {
      const auto row_offset = static_cast<std::size_t>(y) * msg.step;
      for (uint32_t x = roi.x_begin; x < roi.x_end; ++x) {
        const auto offset = row_offset + static_cast<std::size_t>(x) * bytes_per_pixel;
        if (offset + bytes_per_pixel > msg.data.size()) {
          continue;
        }

        const auto depth = depth_meters(msg, offset);
        if (std::isfinite(depth) && min_valid_depth <= depth && depth <= max_valid_depth) {
          stats.min_depth_m = std::min(stats.min_depth_m, depth);
          ++stats.valid_pixels;
        }
      }
    }

    if (stats.valid_pixels == 0) {
      stats.min_depth_m = std::numeric_limits<double>::quiet_NaN();
    }
    return stats;
  }

  RoiBounds roi_bounds(uint32_t width, uint32_t height) const
  {
    const auto fraction_width = clamp_fraction(get_parameter("roi_center_fraction_width").as_double());
    const auto fraction_height = clamp_fraction(get_parameter("roi_center_fraction_height").as_double());
    const auto roi_width = std::max<uint32_t>(1, static_cast<uint32_t>(std::round(width * fraction_width)));
    const auto roi_height = std::max<uint32_t>(1, static_cast<uint32_t>(std::round(height * fraction_height)));
    const auto x_begin = (width - roi_width) / 2U;
    const auto y_begin = (height - roi_height) / 2U;
    return {x_begin, x_begin + roi_width, y_begin, y_begin + roi_height};
  }

  std::pair<bool, double> age_seconds(double seconds) const
  {
    if (!has_last_message_time_) {
      return {false, 0.0};
    }
    return {true, seconds - last_message_time_};
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

  static double clamp_fraction(double value)
  {
    if (!std::isfinite(value)) {
      return 1.0;
    }
    return std::clamp(value, 0.0, 1.0);
  }

  static bool is_supported_encoding(const std::string & encoding)
  {
    return encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      encoding == sensor_msgs::image_encodings::MONO16 ||
      encoding == sensor_msgs::image_encodings::TYPE_32FC1;
  }

  static double depth_meters(const Image & msg, std::size_t offset)
  {
    if (msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      float value = 0.0F;
      std::memcpy(&value, msg.data.data() + offset, sizeof(value));
      return static_cast<double>(value);
    }

    uint16_t value = 0;
    std::memcpy(&value, msg.data.data() + offset, sizeof(value));
    return static_cast<double>(value) * 0.001;
  }

  std::string depth_image_topic_;
  std::string sensor_hardware_id_;
  std::string navigation_hardware_id_;
  double publish_period_{1.0};
  double stale_timeout_{3.0};
  double frequency_window_{5.0};
  double warn_min_frequency_hz_{5.0};

  Image::SharedPtr last_image_;
  bool has_last_message_time_{false};
  double last_message_time_{0.0};
  std::deque<double> frequency_timestamps_;

  rclcpp::Subscription<Image>::SharedPtr subscription_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontObstacleDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
