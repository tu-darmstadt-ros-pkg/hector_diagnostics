#pragma once

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

#include <diagnostic_aggregator/analyzer.h>

namespace hector_software_monitor
{
/*!
 * @brief The SimpleAnalyzer class is a simple diagnostic analyzer that can be used as a blueprint for other
 * analyzers
 *
 * SimpleAnalyzer provides only one matching criterion ('startswith') and removes the prefix for every status item.
 * Required parameters:
 * - \b type This is the class name of the analyzer, used to load the correct plugin type.
 * - \b path All diagnostic items analyzed by the SimpleAnalyzer will be under "Base Path/My Path".
 * - \b find_prefix Parameter for matching, item name must start with this value
 * Optional parameters:
 * - \b timeout Any item that doesn't update within the timeout will be marked as "Stale", and will cause an error
 * in the top-level status. Default is 5.0 seconds. Any value <0 will cause stale items to be ignored. Other
 * matching criteria like 'contains', 'name' etc. can be easily implemented as for the GenericAnalyzer.
 */
class SimpleAnalyzer : public diagnostic_aggregator::Analyzer
{
public:
  /*!
   * @brief Initializes SimpleAnalyzer from namespace, loads parameters
   *
   * @param base_path Prefix for all analyzers (ex: 'Robot')
   * @param nh NodeHandle in full namespace
   * @return True if initialization succeed, false otherwise
   */
  // NOLINTNEXTLINE
  bool init(const std::string base_path, const ros::NodeHandle& nh) override;

  /*!
   * @brief Returns true if item matches any of the given criteria, in this case starts with the given prefix
   *
   */
  // NOLINTNEXTLINE
  bool match(const std::string name) override;

  /*!
   * @brief Update state with new StatusItem
   */
  // NOLINTNEXTLINE
  bool analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item) override;

  /*!
   * @brief Reports current state, returns vector of formatted status messages
   *
   * This method does not check any conditions but serves only to illustrate how report() can be used.
   * @return Vector of DiagnosticStatus messages, with correct prefix for all names.
   */
  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> report() override;

  std::string getPath() const override
  {
    return path_;
  }
  std::string getName() const override
  {
    return nice_name_;
  }

private:
  /*!
   * @brief Stores items by name. State of analyzer
   */
  std::map<std::string, boost::shared_ptr<diagnostic_aggregator::StatusItem>> items_;

  std::string path_;
  std::string nice_name_;
  std::string prefix_;
  double timeout_ = 5.0;  // Default timeout: If no new matched messages arrive within this period the analyzer reports
                          // all stale. This can be the case e.g. if frequency_publisher node is not running.
};                        // namespace diagnostic_aggregator::Analyzer
}  // namespace hector_software_monitor
