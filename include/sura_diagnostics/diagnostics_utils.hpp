#pragma once

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/key_value.hpp"

namespace sura_diagnostics
{

inline bool finite_value(double value)
{
  return std::isfinite(value);
}

inline bool finite_values(const std::vector<double> & values)
{
  for (const auto value : values) {
    if (!finite_value(value)) {
      return false;
    }
  }
  return true;
}

template<typename ArrayT>
std::vector<double> covariance_diagonal_values(const ArrayT & covariance, const std::size_t size)
{
  std::vector<double> values;
  values.reserve(size);
  for (std::size_t index = 0; index < size; ++index) {
    values.push_back(static_cast<double>(covariance[index * size + index]));
  }
  return values;
}

inline diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

inline std::string bool_string(bool value)
{
  return value ? "true" : "false";
}

inline std::string fixed_string(double value, int precision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

inline std::string optional_fixed_string(bool has_value, double value, int precision)
{
  if (!has_value) {
    return "none";
  }
  return fixed_string(value, precision);
}

template<typename T>
std::string numeric_string(T value)
{
  return std::to_string(value);
}

}  // namespace sura_diagnostics
