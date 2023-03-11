#pragma once

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

#include <diagnostic_aggregator/analyzer.h>

namespace hector_software_monitor
{
/**
 * @brief The TopicFrequencyAnalyzer class analyzes the publish rate of topics and compares it to given min and max
 * threshold
 * @details To publish the required diagnostic messages run the frequency_publisher node.
 */
class TopicFrequencyAnalyzer : public diagnostic_aggregator::Analyzer
{
public:
  /**
   * @brief The TopicFrequency struct keeps track of the current and desired frequency of a topic
   * @param last_update Timestamp when this topic received its last update
   * @param avg The publish rate of the topic averaged over a rolling window
   * @param min_frequency_required The minimum required publish rate
   * @param max_frequency_required The maximum required publish rate
   * @param stale_timeout Timeout for marking a topic as stale
   */
  struct TopicFrequency
  {
    TopicFrequency(double min, double max, double stale_timeout);

    ros::Time last_update;
    double avg;
    double min_frequency_required, max_frequency_required;
    double stale_timeout;
  };

  /**
   * @brief Loads data from param server and stores them as TopicFrequency objects.
   * @param base_path passed to base class function
   * @param nh NodeHandle passed to base class function
   * @returns true if successful
   */
  // NOLINTNEXTLINE
  bool init(const std::string base_path, const ros::NodeHandle& nh) override;

  /**
   * @brief Returns true if the StatusItem name starts with prefix
   * @param name the prefix
   * @return true if match found
   */
  // NOLINTNEXTLINE
  bool match(const std::string name) override;

  /**
   * @brief Update state with new StatusItem
   */
  // NOLINTNEXTLINE
  bool analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item) override;

  /**
   * @brief Updates the accumulators of the TopicFrequency objects and compares the averaged frequency to the desired.
   * @return A vector of messages with according reports.
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
   *\brief Stores the status item. This analyzer receives only one item (message) per second so we do not need a map to
   *store multiple items
   */
  boost::shared_ptr<diagnostic_aggregator::StatusItem> item_;

  /**
   * @brief Contains pairs of topic name and belongig desired frequency & accumulated frequency
   */
  std::map<std::string, TopicFrequency> topics_;

  std::string path_;
  std::string nice_name_;
  std::string match_name_;
  ros::Time last_analyzed_msg_;
  double timeout_ = 5.0;  // Default timeout: If no new matched messages arrive within this period the analyzer reports
                          // all stale. This can be the case e.g. if frequency_publisher node is not running.
};

}  // namespace hector_software_monitor
