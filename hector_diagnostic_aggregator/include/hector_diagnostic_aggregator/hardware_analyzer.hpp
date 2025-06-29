
/*!
 * \author Jonathan Lichtenfeld
 */

#ifndef DIAGNOSTIC_AGGREGATOR__GENERIC_ANALYZER_HPP_
#define DIAGNOSTIC_AGGREGATOR__GENERIC_ANALYZER_HPP_

#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_aggregator/analyzer.hpp"
#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"

#include "hector_diagnostic_aggregator/hardware_analyzer_base.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diagnostic_aggregator
{
/*!
 *\brief Returns list of strings from a parameter
 *
 * Given an XmlRpcValue, gives vector of strings of that parameter
 *\return False if XmlRpcValue is not string or array of strings
 */
inline bool getParamVals(rclcpp::Parameter param, std::vector<std::string> & output)
{
  rclcpp::ParameterType type = param.get_type();
  if (type == rclcpp::ParameterType::PARAMETER_STRING) {
    std::string find = param.as_string();
    output.push_back(find);
    return true;
  } else if (type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    output = param.as_string_array();
    return true;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger("generic_analyzer"),
    "Parameter not a list or string, unable to return values. Parameter type: %s",
    param.get_type_name().c_str());
  output.clear();
  return false;
}


class HardwareAnalyzer : public HardwareAnalyzerBase
{
public:

  HardwareAnalyzer();

  virtual ~HardwareAnalyzer();

  bool init(
    const std::string & base_path, const std::string & breadcrumb,
    const rclcpp::Node::SharedPtr node);

  virtual std::vector<std::shared_ptr<diagnostic_msgs::msg::DiagnosticStatus>> report();

  virtual bool match(const std::string & name);

private:
  std::vector<std::string> chaff_;
  std::vector<std::string> expected_;
  std::vector<std::string> startswith_;
  std::vector<std::string> contains_;
  std::vector<std::string> name_;
  std::vector<std::regex> regex_;
};

}  // namespace diagnostic_aggregator

#endif  // HECTOR_DIAGNOSTIC_AGGREGATOR__HARDWARE_ANALYZER_HPP_