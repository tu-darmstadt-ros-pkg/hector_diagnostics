/**!< \author Jonathan Lichtenfeld */

#ifndef DIAGNOSTIC_AGGREGATOR__HARDWARE_ANALYZER_BASE_HPP_
#define DIAGNOSTIC_AGGREGATOR__HARDWARE_ANALYZER_BASE_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_aggregator/analyzer.hpp"
#include "diagnostic_aggregator/generic_analyzer_base.hpp"
#include "diagnostic_aggregator/status_item.hpp"
#include "diagnostic_aggregator/visibility_control.hpp"

#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"

#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"

namespace diagnostic_aggregator
{

class HardwareAnalyzerBase : public GenericAnalyzerBase
{
public:
  HardwareAnalyzerBase() = default;
  virtual ~HardwareAnalyzerBase() = default;

  bool analyze(const std::shared_ptr<StatusItem> item) override {
    if (item->getHwId() != hardware_id_) {
        RCLCPP_DEBUG(
        rclcpp::get_logger("HardwareAnalyzerBase"),
        "Skipping item '%s': expected hw_id='%s', got '%s'",
        item->getName().c_str(), hardware_id_.c_str(), item->getHwId().c_str());
        return false;
  }

  // Call GenericAnalyzerBase::analyze() for everything else
  return GenericAnalyzerBase::analyze(item);
}


protected:
  // The hardware_id that this analyzer is responsible for.
  std::string hardware_id_;
};

}  // namespace diagnostic_aggregator

#endif  // DIAGNOSTIC_AGGREGATOR__HARDWARE_ANALYZER_BASE_HPP_