#pragma once

#include <ros/ros.h>
#include <rosgraph_msgs/TopicStatistics.h>
#include <topic_tools/shape_shifter.h>

namespace hector_software_monitor
{
/**
 * @brief The TopicFrequencyChecker class publishs a DiagnosticStatus containing the topics corresponding publish rates
 * specified in a config file
 */
class TopicFrequencyChecker
{
public:
  /**
   * @brief The Connection class keeps track of the number of messages sent over a connection (publisher/subscriber
   * pair) and the timestamp of last evaluation
   */
  struct Connection
  {
    Connection();

    double frequency;
    ros::Time last_msg_received;
    ros::Duration last_update_interval;  // Time between the last two updates on /statistics
    size_t delivered_msgs;
    size_t dropped_msgs;
  };

  /**
   * @brief The TopicData struct stores data for each subscriber of a topic
   * evaluation
   */
  struct TopicData
  {
    TopicData(double min, double max, double timeout);

    /**
     * @brief data Key: pair of publisher/subscriber. Value: data for this connection
     */
    std::map<std::pair<std::string, std::string>, Connection> connections;
    double min_frequency_required, max_frequency_required;
    /**
     * @brief timeout If no statistics msg was received after this timeout -> label stale
     */
    double timeout;
    bool initialized;
    ros::Time last_msg_received;  // only used for topics with max desired freq == 0
  };

  TopicFrequencyChecker();
  ~TopicFrequencyChecker() = default;

private:
  /**
   * @brief topicCallback A generic callback function shared by all subscribers
   * @param msg The message
   * @param topic The topic on which the message was received
   */
  void statCallback(const rosgraph_msgs::TopicStatisticsConstPtr& msg);

  /**
   * @brief timerCallback A method called periodically to evaluate all registered topics for their frequency
   * @param event
   */
  void timerCallback(const ros::TimerEvent& event);

  /**
   * @brief inverseCallback callback for topics that should not be used at all
   */
  void inverseCallback(const topic_tools::ShapeShifter::ConstPtr& input, const std::string& topic);

  /**
   * @brief Contains topics and belongig data
   */
  std::map<std::string, TopicData> topics_;

  ros::Subscriber stats_sub_;
  std::vector<ros::Subscriber> inverse_subs_;  // Subscribers for topics that should not receive anything at all
  ros::Publisher diagnostics_pub_;
  ros::Timer timer_;
};
}  // namespace hector_software_monitor
