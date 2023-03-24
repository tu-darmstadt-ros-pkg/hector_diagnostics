#pragma once

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <ros/ros.h>

#include <topic_tools/shape_shifter.h>

namespace hector_software_monitor
{
namespace ba = boost::accumulators;

/**
 * @brief The TopicFrequencyChecker class publishs a DiagnosticStatus containing the topics corresponding publish rates
 * specified in a config file
 */
class TopicFrequencyChecker
{
public:
  /**
   * @brief The TopicData struct keeps track of the number of messages received on a topic and the timestamp of last
   * evaluation
   */
  struct TopicData
  {
    TopicData(double min, double max, double timeout);

    ros::Time last_evaluation;
    ros::Time last_msg_received;
    ba::accumulator_set<double, ba::stats<ba::tag::rolling_mean, ba::tag::min, ba::tag::max>> intervals_acc;
    size_t count;
    bool new_msgs_received;
    double min_frequency_required, max_frequency_required;
    double timeout;
  };

  TopicFrequencyChecker();
  ~TopicFrequencyChecker() = default;

private:
  /**
   * @brief topicCallback A generic callback function shared by all subscribers
   * @param msg The message
   * @param topic The topic on which the message was received
   */
  void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic);

  /**
   * @brief frequencyCallback A function that is called periodically to evaluate the frequency of each topic
   */
  void timerCallback(const ros::TimerEvent& event);

  /**
   * @brief Contains pairs of topic name and belongig data
   */
  std::map<std::string, TopicData> topics_;
  std::vector<ros::Subscriber> topic_subs_;

  ros::Time time_prev_;
  ros::Timer calc_timer_;

  ros::Subscriber topic_sub_;
  ros::Publisher frequency_pub_;
};
}  // namespace hector_software_monitor
