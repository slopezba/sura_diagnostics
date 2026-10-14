#include <algorithm>
#include <cctype>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_diagnostics/diagnostics_utils.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using sura_diagnostics::key_value;
using sura_diagnostics::strip_slashes;

namespace
{

std::vector<std::string> split_path(const std::string & value)
{
  std::vector<std::string> parts;
  std::stringstream stream(strip_slashes(value));
  std::string part;
  while (std::getline(stream, part, '/')) {
    if (!part.empty()) {
      parts.push_back(part);
    }
  }
  return parts;
}

std::string join_path(const std::vector<std::string> & parts, std::size_t first)
{
  std::string result;
  for (std::size_t index = first; index < parts.size(); ++index) {
    if (!result.empty()) {
      result += "/";
    }
    result += parts[index];
  }
  return result;
}

std::string upper_string(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::toupper(character));
  });
  return value;
}

unsigned char max_level(unsigned char lhs, unsigned char rhs)
{
  return lhs > rhs ? lhs : rhs;
}

std::string level_message(unsigned char level)
{
  switch (level) {
    case DiagnosticStatus::OK:
      return "OK";
    case DiagnosticStatus::WARN:
      return "WARN";
    case DiagnosticStatus::ERROR:
      return "ERROR";
    case DiagnosticStatus::STALE:
      return "STALE";
    default:
      return "UNKNOWN";
  }
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

bool is_robot_status(const DiagnosticStatus & status, const std::vector<std::string> & parts)
{
  if (parts.size() < 2) {
    return false;
  }
  const auto robot_prefix = parts[0] + "_";
  return starts_with(status.hardware_id, robot_prefix);
}

struct StoredStatus
{
  DiagnosticStatus status;
  rclcpp::Time stamp;
};

struct GroupState
{
  unsigned char level{DiagnosticStatus::OK};
  int ok_count{0};
  int warn_count{0};
  int error_count{0};
  int stale_count{0};

  void add(unsigned char item_level)
  {
    level = max_level(level, item_level);
    if (item_level == DiagnosticStatus::OK) {
      ++ok_count;
    } else if (item_level == DiagnosticStatus::WARN) {
      ++warn_count;
    } else if (item_level == DiagnosticStatus::ERROR) {
      ++error_count;
    } else if (item_level == DiagnosticStatus::STALE) {
      ++stale_count;
    }
  }

  int total() const
  {
    return ok_count + warn_count + error_count + stale_count;
  }
};

DiagnosticStatus group_status(const std::string & name, const GroupState & state)
{
  DiagnosticStatus status;
  status.name = name;
  status.hardware_id = strip_slashes(name);
  status.level = state.total() > 0 ? state.level : DiagnosticStatus::STALE;
  status.message = state.total() > 0 ? level_message(status.level) : "No diagnostics received";
  status.values = {
    key_value("ok_count", std::to_string(state.ok_count)),
    key_value("warn_count", std::to_string(state.warn_count)),
    key_value("error_count", std::to_string(state.error_count)),
    key_value("stale_count", std::to_string(state.stale_count)),
    key_value("total_count", std::to_string(state.total())),
  };
  return status;
}

}  // namespace

class GlobalDiagnosticsAggregator : public rclcpp::Node
{
public:
  GlobalDiagnosticsAggregator()
  : Node("global_diagnostics_aggregator")
  {
    declare_parameter<std::string>("root_path", "ROBOTS");
    declare_parameter<double>("pub_rate", 1.0);
    declare_parameter<double>("stale_timeout", 5.0);

    root_path_ = "/" + strip_slashes(get_parameter("root_path").as_string());
    pub_rate_ = get_parameter("pub_rate").as_double();
    stale_timeout_ = get_parameter("stale_timeout").as_double();

    diagnostics_sub_ = create_subscription<DiagnosticArray>(
      "/diagnostics", 100, [this](const DiagnosticArray::SharedPtr msg) {
        on_diagnostics(msg);
      });
    diagnostics_pub_ = create_publisher<DiagnosticArray>("/diagnostics_agg", 10);
    top_level_pub_ = create_publisher<DiagnosticStatus>("/diagnostics_toplevel_state", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(pub_rate_, 0.1)),
      [this]() { publish_aggregated(); });
  }

private:
  void on_diagnostics(const DiagnosticArray::SharedPtr msg)
  {
    const auto now = get_clock()->now();
    bool area_level_changed = false;
    for (const auto & status : msg->status) {
      const auto parts = split_path(status.name);
      if (!is_robot_status(status, parts)) {
        continue;
      }
      const auto name = canonical_name(status.name);
      const auto previous = statuses_.find(name);
      if (parts.size() >= 3 && parts[1] == "Navigation" && parts[2] == "AreaLimit" &&
        (previous == statuses_.end() || previous->second.status.level != status.level))
      {
        area_level_changed = true;
      }
      statuses_[name] = StoredStatus{status, now};
    }
    if (area_level_changed) {
      publish_aggregated();
    }
  }

  void publish_aggregated()
  {
    const auto now = get_clock()->now();
    std::map<std::string, GroupState> robot_states;
    std::map<std::string, GroupState> category_states;
    GroupState root_state;
    std::vector<DiagnosticStatus> leaf_statuses;

    for (const auto & [name, stored] : statuses_) {
      auto parts = split_path(name);
      if (parts.size() < 2) {
        continue;
      }

      const auto robot = parts[0];
      const auto robot_path = root_path_ + "/" + upper_string(robot);
      const auto category_path = robot_path + "/" + parts[1];
      const auto leaf_path = robot_path + "/" + join_path(parts, 1);

      auto status = stored.status;
      status.name = leaf_path;
      status.values.push_back(key_value("source_name", name));

      if ((now - stored.stamp).seconds() > stale_timeout_) {
        status.level = DiagnosticStatus::STALE;
        status.message = "Diagnostic status is stale";
      }

      robot_states[robot_path].add(status.level);
      category_states[category_path].add(status.level);
      root_state.add(status.level);
      leaf_statuses.push_back(status);
    }

    DiagnosticArray array_msg;
    array_msg.header.stamp = now;
    array_msg.status.push_back(group_status(root_path_, root_state));

    for (const auto & [name, state] : robot_states) {
      array_msg.status.push_back(group_status(name, state));
    }
    for (const auto & [name, state] : category_states) {
      array_msg.status.push_back(group_status(name, state));
    }

    std::sort(leaf_statuses.begin(), leaf_statuses.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.name < rhs.name;
    });
    array_msg.status.insert(
      array_msg.status.end(), leaf_statuses.begin(), leaf_statuses.end());

    diagnostics_pub_->publish(array_msg);
    top_level_pub_->publish(array_msg.status.front());
  }

  static std::string canonical_name(const std::string & name)
  {
    return "/" + strip_slashes(name);
  }

  std::string root_path_{"/ROBOTS"};
  double pub_rate_{1.0};
  double stale_timeout_{5.0};
  std::map<std::string, StoredStatus> statuses_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr top_level_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalDiagnosticsAggregator>());
  rclcpp::shutdown();
  return 0;
}
