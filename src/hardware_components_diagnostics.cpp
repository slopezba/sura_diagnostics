#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_hardware_components.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"

using controller_manager_msgs::srv::ListHardwareComponents;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using controller_manager_msgs::msg::HardwareComponentState;
using lifecycle_msgs::msg::State;
using sura_diagnostics::fixed_string;
using sura_diagnostics::key_value;
using sura_diagnostics::optional_fixed_string;
using sura_diagnostics::prefixed_diagnostic_name;

class HardwareComponentsDiagnostics : public rclcpp::Node
{
public:
  HardwareComponentsDiagnostics()
  : Node("hardware_components_diagnostics")
  {
    declare_parameter<std::string>(
      "list_hardware_components_service",
      "");
    declare_parameter<double>("publish_period", 1.0);
    declare_parameter<double>("service_timeout", 2.0);
    declare_parameter<std::string>("robot_namespace", "");
    declare_parameter<std::vector<std::string>>(
      "component_names", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>(
      "diagnostic_names", {"/Hardware/Thrusters", "/Hardware/Left Arm", "/Hardware/Right Arm"});

    service_name_ = get_parameter("list_hardware_components_service").as_string();
    publish_period_ = get_parameter("publish_period").as_double();
    service_timeout_ = get_parameter("service_timeout").as_double();
    robot_namespace_ = get_parameter("robot_namespace").as_string();
    component_names_ = get_parameter("component_names").as_string_array();
    diagnostic_names_ = get_parameter("diagnostic_names").as_string_array();

    if (diagnostic_names_.size() != component_names_.size()) {
      RCLCPP_WARN(
        get_logger(),
        "diagnostic_names size does not match component_names; using component names");
      diagnostic_names_.clear();
      for (const auto & name : component_names_) {
        diagnostic_names_.push_back("/Hardware/" + name);
      }
    }

    client_ = create_client<ListHardwareComponents>(service_name_);
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() { on_timer(); });
  }

private:
  void on_timer()
  {
    const auto now = get_clock()->now();
    const auto seconds = time_seconds(now);
    collect_response(seconds);
    publish_diagnostics(now, seconds);
    request_components(seconds);
  }

  void collect_response(double seconds)
  {
    if (!pending_request_.valid()) {
      return;
    }
    if (pending_request_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      const auto response = pending_request_.get();
      last_components_.clear();
      for (const auto & component : response->component) {
        last_components_[component.name] = component;
      }
      pending_request_ = {};
      has_request_start_seconds_ = false;
      last_response_seconds_ = seconds;
      has_last_response_seconds_ = true;
      return;
    }
    if (has_request_start_seconds_ && seconds - request_start_seconds_ > service_timeout_) {
      pending_request_ = {};
      has_request_start_seconds_ = false;
    }
  }

  void request_components(double seconds)
  {
    if (pending_request_.valid() || !client_->service_is_ready()) {
      return;
    }
    auto future_and_id =
      client_->async_send_request(std::make_shared<ListHardwareComponents::Request>());
    pending_request_ = future_and_id.future.share();
    request_start_seconds_ = seconds;
    has_request_start_seconds_ = true;
  }

  void publish_diagnostics(const rclcpp::Time & now, double seconds)
  {
    DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    for (std::size_t index = 0; index < component_names_.size(); ++index) {
      array_msg.status.push_back(
        build_status(component_names_[index], diagnostic_names_[index], seconds));
    }
    diagnostics_pub_->publish(array_msg);
  }

  DiagnosticStatus build_status(
    const std::string & component_name, const std::string & diagnostic_name, double seconds)
  {
    DiagnosticStatus status;
    status.name = prefixed_diagnostic_name(robot_namespace_, diagnostic_name);
    status.hardware_id = component_name;

    const auto component = last_components_.find(component_name);
    const auto has_age = has_last_response_seconds_;
    const auto age_seconds = has_age ? seconds - last_response_seconds_ : 0.0;

    unsigned char state_id = State::PRIMARY_STATE_UNKNOWN;
    std::string state_label = "unknown";
    std::string component_type = "unknown";
    std::string class_type = "unknown";

    if (component == last_components_.end()) {
      status.level = DiagnosticStatus::STALE;
      status.message = "Hardware component state is unavailable";
    } else {
      state_id = component->second.state.id;
      state_label = component->second.state.label;
      component_type = component->second.type;
      class_type = component->second.class_type;
      auto evaluation = evaluate_state(state_id, state_label);
      status.level = evaluation.first;
      status.message = evaluation.second;
    }

    status.values = {
      key_value("component_name", component_name),
      key_value("state_id", std::to_string(state_id)),
      key_value("state_label", state_label),
      key_value("component_type", component_type),
      key_value("class_type", class_type),
      key_value("service", service_name_),
      key_value("age_seconds", optional_fixed_string(has_age, age_seconds, 3)),
    };
    return status;
  }

  std::pair<unsigned char, std::string> evaluate_state(
    unsigned char state_id, const std::string & state_label) const
  {
    auto normalized_label = state_label;
    std::transform(normalized_label.begin(), normalized_label.end(), normalized_label.begin(), ::tolower);

    if (state_id == State::PRIMARY_STATE_ACTIVE || normalized_label == "active") {
      return {DiagnosticStatus::OK, "Hardware component is active"};
    }
    if (
      state_id == State::PRIMARY_STATE_INACTIVE || normalized_label == "inactive" ||
      normalized_label == "configured")
    {
      return {DiagnosticStatus::WARN, "Hardware component is configured but inactive"};
    }
    if (
      state_id == State::PRIMARY_STATE_UNCONFIGURED ||
      state_id == State::TRANSITION_STATE_CLEANINGUP ||
      normalized_label == "unconfigured" || normalized_label == "cleaningup" ||
      normalized_label == "cleanup")
    {
      return {DiagnosticStatus::ERROR, "Hardware component is in cleanup"};
    }
    return {DiagnosticStatus::ERROR, "Hardware component state is " + state_label};
  }

  static double time_seconds(const rclcpp::Time & time)
  {
    return static_cast<double>(time.nanoseconds()) / 1e9;
  }

  std::string service_name_;
  std::string robot_namespace_;
  double publish_period_{1.0};
  double service_timeout_{2.0};
  std::vector<std::string> component_names_;
  std::vector<std::string> diagnostic_names_;
  rclcpp::Client<ListHardwareComponents>::SharedPtr client_;
  rclcpp::Client<ListHardwareComponents>::SharedFuture pending_request_;
  bool has_request_start_seconds_{false};
  double request_start_seconds_{0.0};
  std::map<std::string, HardwareComponentState> last_components_;
  bool has_last_response_seconds_{false};
  double last_response_seconds_{0.0};
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareComponentsDiagnostics>());
  rclcpp::shutdown();
  return 0;
}
