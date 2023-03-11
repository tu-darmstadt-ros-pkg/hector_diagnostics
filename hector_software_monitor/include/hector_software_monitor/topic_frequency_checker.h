#pragma once

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
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
   * @brief The TimeFrequency struct keeps track of the number of messages received on a topic and the timestamp of last
   * evaluation
   */
  struct TimeFrequency
  {
    TimeFrequency();

    ros::Time last_evaluation;
    ros::Time last_msg_received;
    ba::accumulator_set<double, ba::stats<ba::tag::rolling_mean, ba::tag::min, ba::tag::max, ba::tag::count>>
        intervals_acc;
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
  void frequencyCallback(const ros::TimerEvent& event);

  /**
   * @brief Contains pairs of topic name and belongig timestamp & frequency
   */
  std::map<std::string, TimeFrequency> topics_;
  std::vector<ros::Subscriber> topic_subs_;

  ros::Time time_prev_;
  ros::Timer calc_timer_;

  ros::Subscriber topic_sub_;
  ros::Publisher frequency_pub_;
};
}  // namespace hector_software_monitor
