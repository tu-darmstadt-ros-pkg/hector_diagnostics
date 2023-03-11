#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <hector_software_monitor/topic_frequency_checker.h>

namespace hector_software_monitor
{
TopicFrequencyChecker::TimeFrequency::TimeFrequency()
  : last_evaluation(ros::Time::now())
  , last_msg_received(ros::Time(0.0))
  , intervals_acc(ba::tag::rolling_window::window_size = 100)
{
}

TopicFrequencyChecker::TopicFrequencyChecker()
{
  ros::NodeHandle nh;

  // Load topics from parameter server
  XmlRpc::XmlRpcValue topic_frequency_list;
  if (!nh.getParam("topic_frequency_analyzer", topic_frequency_list))
  {
    ROS_ERROR("[TopicFrequencyChecker] Could not get \"topic_frequency_analyzer\" from param server");
  }
  if (topic_frequency_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("[TopicFrequencyChecker] Parameter topic_frequency_analyzer must be a list of dicts");
  }

  // Iterate over all given topics
  // NOLINTNEXTLINE
  for (std::size_t i = 0; i < topic_frequency_list.size(); i++)
  {
    const XmlRpc::XmlRpcValue& dict = topic_frequency_list[i];

    if (dict.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("[TopicFrequencyChecker] Parameter topic_frequency_analyzer must be a list of dicts");
    }

    if (!(dict.hasMember("topic") && dict["topic"].getType() == XmlRpc::XmlRpcValue::TypeString))
    {
      ROS_ERROR("[TopicFrequencyChecker] Topic name was not specified");
    }

    // Save topic in map
    std::string topic = static_cast<std::string>(dict["topic"]);
    if (topic[0] != '/')
      topic = '/' + topic;
    TimeFrequency time_freq;
    topics_.insert(std::pair<std::string, TimeFrequency>(topic, time_freq));

    // Setup subscriber on topic
    ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(
        topic, 100, boost::bind(&TopicFrequencyChecker::topicCallback, this, _1, topic));
    topic_subs_.push_back(sub);
  }

  frequency_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100);

  // Set up a timer that triggers frequencyCallback() at a fixed rate
  calc_timer_ = nh.createTimer(ros::Duration(1.0), &TopicFrequencyChecker::frequencyCallback, this);
}

void TopicFrequencyChecker::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic)
{
  ros::Time stamp = ros::Time::now();

  // Wait for second message
  if (topics_[topic].last_msg_received == ros::Time(0.0))
  {
    topics_[topic].last_msg_received = stamp;
    return;
  }

  ros::Duration interval = stamp - topics_[topic].last_msg_received;  // TODO replace stamp with msg->header.stamp
  topics_[topic].intervals_acc(interval.toSec());
  topics_[topic].last_msg_received = stamp;
}

void TopicFrequencyChecker::frequencyCallback(const ros::TimerEvent& event)
{
  diagnostic_msgs::DiagnosticArray diag_array;
  diagnostic_msgs::DiagnosticStatus diag_status;
  diag_status.name = "topic_frequency_status";

  for (auto& topic : topics_)
  {
    diagnostic_msgs::KeyValue keyval_avg;
    diagnostic_msgs::KeyValue keyval_min;
    diagnostic_msgs::KeyValue keyval_max;
    diagnostic_msgs::KeyValue keyval_count;

    keyval_avg.key = topic.first + "_avg";
    keyval_min.key = topic.first + "_min";
    keyval_max.key = topic.first + "_max";
    keyval_count.key = topic.first + "_count";

    if (topic.second.last_evaluation > topic.second.last_msg_received)
      keyval_avg.value = "no new messages";
    else
      keyval_avg.value = std::to_string(1.0 / ba::rolling_mean(topic.second.intervals_acc));
    keyval_min.value = std::to_string(ba::min(topic.second.intervals_acc)) + 's';
    keyval_max.value = std::to_string(ba::max(topic.second.intervals_acc)) + 's';
    keyval_count.value = std::to_string(ba::count(topic.second.intervals_acc) + 1);  // #messages = #intervals + 1

    // update timestamp
    topic.second.last_evaluation = event.current_real;

    diag_status.values.push_back(keyval_avg);
    diag_status.values.push_back(keyval_min);
    diag_status.values.push_back(keyval_max);
    diag_status.values.push_back(keyval_count);
  }

  diag_array.status.push_back(diag_status);
  diag_array.header.stamp = ros::Time::now();
  frequency_pub_.publish(diag_array);
}
}  // namespace hector_software_monitor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_frequency_checker");
  hector_software_monitor::TopicFrequencyChecker topic_frequency_checker;
  ros::spin();
  return 0;
}
