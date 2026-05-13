#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"
#include "sura_msgs/msg/controller_debug.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using rcl_interfaces::msg::ParameterType;
using rcl_interfaces::srv::GetParameters;
using sura_msgs::msg::ControllerDebug;
using sura_diagnostics::bool_string;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;

struct ControllerState
{
  ControllerDebug msg;
  rclcpp::Time last_receive_time;
};

class ControllerDebugDiagnostics : public rclcpp::Node
{
public:
  ControllerDebugDiagnostics()
  : Node("controller_debug_diagnostics")
  {
    declare_parameter<std::string>("debug_topic", "/cirtesub/controller/debug");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("stale_timeout", 2.5);
    declare_parameter<double>("controller_frequency_hz", 100.0);
    declare_parameter<bool>("use_controller_manager_update_rate", true);
    declare_parameter<std::string>(
      "controller_manager_node", "/cirtesub/controller/controller_manager");
    declare_parameter<double>("update_rate_refresh_period", 5.0);
    declare_parameter<double>("warning_time_margin_us", 2000.0);

    debug_topic_ = get_parameter("debug_topic").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();
    controller_frequency_hz_ = get_parameter("controller_frequency_hz").as_double();
    use_controller_manager_update_rate_ =
      get_parameter("use_controller_manager_update_rate").as_bool();
    controller_manager_node_ = get_parameter("controller_manager_node").as_string();
    update_rate_refresh_period_ = get_parameter("update_rate_refresh_period").as_double();
    warning_time_margin_us_ = get_parameter("warning_time_margin_us").as_double();

    subscription_ = create_subscription<ControllerDebug>(
      debug_topic_, 10,
      [this](const ControllerDebug::SharedPtr msg) { on_controller_debug(msg); });
    update_rate_client_ = create_client<GetParameters>(
      controller_manager_node_ + "/get_parameters");
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() { publish_diagnostics(); });

    if (use_controller_manager_update_rate_) {
      update_rate_timer_ = create_wall_timer(
        std::chrono::duration<double>(update_rate_refresh_period_),
        [this]() { request_controller_manager_update_rate(); });
      request_controller_manager_update_rate();
    }
  }

private:
  void on_controller_debug(const ControllerDebug::SharedPtr msg)
  {
    const auto controller_name = trim(msg->controller_name);
    if (controller_name.empty()) {
      RCLCPP_WARN(get_logger(), "Received ControllerDebug message without controller_name");
      return;
    }
    states_[controller_name] = ControllerState{*msg, get_clock()->now()};
  }

  static std::string display_controller_name(const std::string & controller_name)
  {
    const auto trimmed = trim_slashes(controller_name);
    const auto separator = trimmed.find_last_of('/');
    if (separator == std::string::npos) {
      return trimmed;
    }
    return trimmed.substr(separator + 1);
  }

  void publish_diagnostics()
  {
    const auto now = get_clock()->now();
    DiagnosticArray array_msg;
    array_msg.header.stamp = now;

    std::vector<DiagnosticStatus> statuses;
    uint64_t total_deadline_misses = 0;
    int active_count = 0;
    int stale_count = 0;
    double worst_last_update_us = 0.0;
    double worst_max_update_us = 0.0;
    std::string worst_controller_name;
    unsigned char summary_level = DiagnosticStatus::OK;
    std::string summary_message = "Controller diagnostics healthy";

    for (const auto & item : states_) {
      auto result = build_controller_status(item.first, item.second, now);
      statuses.push_back(result.first);
      const auto is_stale = result.second;

      total_deadline_misses += item.second.msg.deadline_miss_count;
      if (item.second.msg.active) {
        ++active_count;
      }
      if (is_stale) {
        ++stale_count;
      }
      if (item.second.msg.last_update_us >= worst_last_update_us) {
        worst_last_update_us = item.second.msg.last_update_us;
        worst_controller_name = item.first;
      }
      if (item.second.msg.max_update_us >= worst_max_update_us) {
        worst_max_update_us = item.second.msg.max_update_us;
      }
      summary_level = std::max(summary_level, result.first.level);
    }

    if (statuses.empty()) {
      summary_level = DiagnosticStatus::WARN;
      summary_message = "No controller debug data received";
    } else if (stale_count > 0) {
      summary_message = "Some controller debug data is stale";
    } else if (total_deadline_misses > 0) {
      summary_message = "Controllers reported deadline misses";
    } else if (active_count == 0) {
      summary_message = "No active controllers reporting debug data";
    }

    statuses.push_back(build_summary_status(
      summary_level, summary_message, states_.size(), active_count, stale_count,
      total_deadline_misses, worst_controller_name, worst_last_update_us,
      worst_max_update_us));

    array_msg.status = statuses;
    diagnostics_pub_->publish(array_msg);
  }

  std::pair<DiagnosticStatus, bool> build_controller_status(
    const std::string & controller_name, const ControllerState & state,
    const rclcpp::Time & now)
  {
    const auto elapsed = now - state.last_receive_time;
    const auto is_stale = elapsed.seconds() > stale_timeout_;
    const auto & msg = state.msg;

    DiagnosticStatus status;
    status.name = "/Controllers/" + display_controller_name(controller_name);
    status.hardware_id = "cirtesub_control_stack";

    const auto configured_period_us = controller_period_us();
    const auto reported_period_us = msg.desired_period_us;
    const auto desired_period_us =
      reported_period_us > 0.0 ? reported_period_us : configured_period_us;
    const auto warning_limit_us = std::max(0.0, desired_period_us - warning_time_margin_us_);
    const auto max_update_us = msg.max_update_us;
    const auto margin_to_warning_us = warning_limit_us - max_update_us;
    const auto margin_to_error_us = desired_period_us - max_update_us;

    if (is_stale) {
      status.level = DiagnosticStatus::STALE;
      status.message = "Stale debug data";
    } else if (margin_to_error_us < 0.0 && desired_period_us > 0.0) {
      status.level = DiagnosticStatus::ERROR;
      status.message =
        "Update time exceeded configured period by " +
        fixed_string(std::abs(margin_to_error_us), 3) + " us";
    } else if (msg.deadline_miss_count > 0) {
      status.level = DiagnosticStatus::WARN;
      status.message = "Deadline misses detected";
    } else if (!msg.active) {
      status.level = DiagnosticStatus::OK;
      status.message = "Controller inactive";
    } else if (margin_to_warning_us <= 0.0) {
      status.level = DiagnosticStatus::WARN;
      status.message =
        "Update time close to configured period: " +
        fixed_string(margin_to_error_us, 3) + " us to error";
    } else {
      status.level = DiagnosticStatus::OK;
      status.message =
        "Controller debug healthy: " + fixed_string(margin_to_warning_us, 3) +
        " us to warning, " + fixed_string(margin_to_error_us, 3) + " us to error";
    }

    status.values = {
      key_value("active", bool_string(msg.active)),
      key_value("chained_mode", bool_string(msg.chained_mode)),
      key_value("debug_topic", debug_topic_),
      key_value("controller_frequency_hz", fixed_string(controller_frequency_hz_, 3)),
      key_value("controller_frequency_source", controller_frequency_source_),
      key_value("controller_manager_node", controller_manager_node_),
      key_value("configured_period_us", fixed_string(configured_period_us, 3)),
      key_value("reported_period_us", fixed_string(reported_period_us, 3)),
      key_value("desired_period_us", fixed_string(desired_period_us, 3)),
      key_value("warning_limit_us", fixed_string(warning_limit_us, 3)),
      key_value("margin_to_warning_us", fixed_string(margin_to_warning_us, 3)),
      key_value("margin_to_error_us", fixed_string(margin_to_error_us, 3)),
      key_value("warning_time_margin_us", fixed_string(warning_time_margin_us_, 3)),
      key_value("last_update_us", fixed_string(msg.last_update_us, 3)),
      key_value("avg_update_us", fixed_string(msg.avg_update_us, 3)),
      key_value("max_update_us", fixed_string(msg.max_update_us, 3)),
      key_value("min_update_us", fixed_string(msg.min_update_us, 3)),
      key_value("deadline_miss_count", std::to_string(msg.deadline_miss_count)),
      key_value("cycle_count", std::to_string(msg.cycle_count)),
      key_value("stale_for_ms", fixed_string(static_cast<double>(elapsed.nanoseconds()) / 1e6, 3)),
    };
    return {status, is_stale};
  }

  double controller_period_us() const
  {
    if (controller_frequency_hz_ <= 0.0) {
      return 0.0;
    }
    return 1000000.0 / controller_frequency_hz_;
  }

  void request_controller_manager_update_rate()
  {
    if (update_rate_request_in_flight_ || !update_rate_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<GetParameters::Request>();
    request->names = {"update_rate"};
    update_rate_request_in_flight_ = true;
    update_rate_client_->async_send_request(
      request, [this](rclcpp::Client<GetParameters>::SharedFuture future) {
        on_update_rate_response(future);
      });
  }

  void on_update_rate_response(rclcpp::Client<GetParameters>::SharedFuture future)
  {
    update_rate_request_in_flight_ = false;
    const auto response = future.get();
    if (response->values.empty()) {
      return;
    }
    const auto & value = response->values.front();
    double update_rate = 0.0;
    if (value.type == ParameterType::PARAMETER_INTEGER) {
      update_rate = static_cast<double>(value.integer_value);
    } else if (value.type == ParameterType::PARAMETER_DOUBLE) {
      update_rate = value.double_value;
    }
    if (update_rate > 0.0) {
      controller_frequency_hz_ = update_rate;
      controller_frequency_source_ = "controller_manager.update_rate";
    }
  }

  DiagnosticStatus build_summary_status(
    unsigned char level, const std::string & message, std::size_t total_count,
    int active_count, int stale_count, uint64_t total_deadline_misses,
    const std::string & worst_controller_name, double worst_last_update_us,
    double worst_max_update_us)
  {
    DiagnosticStatus status;
    status.name = "/Controllers/Summary";
    status.hardware_id = "cirtesub_control_stack";
    status.level = level;
    status.message = message;
    status.values = {
      key_value("total_count", std::to_string(total_count)),
      key_value("active_count", std::to_string(active_count)),
      key_value("stale_count", std::to_string(stale_count)),
      key_value("total_deadline_miss_count", std::to_string(total_deadline_misses)),
      key_value("worst_controller_name", worst_controller_name),
      key_value("worst_last_update_us", fixed_string(worst_last_update_us, 3)),
      key_value("worst_max_update_us", fixed_string(worst_max_update_us, 3)),
    };
    return status;
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

  static std::string trim_slashes(const std::string & value)
  {
    const auto first = value.find_first_not_of('/');
    if (first == std::string::npos) {
      return "";
    }
    const auto last = value.find_last_not_of('/');
    return value.substr(first, last - first + 1);
  }

  std::string debug_topic_;
  double publish_period_{1.0};
  double stale_timeout_{2.5};
  double controller_frequency_hz_{100.0};
  std::string controller_frequency_source_{"configured"};
  bool use_controller_manager_update_rate_{true};
  std::string controller_manager_node_;
  double update_rate_refresh_period_{5.0};
  double warning_time_margin_us_{2000.0};
  std::map<std::string, ControllerState> states_;
  bool update_rate_request_in_flight_{false};
  rclcpp::Subscription<ControllerDebug>::SharedPtr subscription_;
  rclcpp::Client<GetParameters>::SharedPtr update_rate_client_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr update_rate_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerDebugDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
