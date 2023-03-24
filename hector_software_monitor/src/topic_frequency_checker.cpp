#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <hector_software_monitor/topic_frequency_checker.h>

namespace hector_software_monitor
{
TopicFrequencyChecker::TopicData::TopicData(double min, double max, double timeout)
  : last_msg_received(ros::Time(0.0))
  , intervals_acc(ba::tag::rolling_window::window_size = 100)
  , count(0)
  , new_msgs_received(false)
  , min_frequency_required(min)
  , max_frequency_required(max)
  , timeout(timeout)
{
}

TopicFrequencyChecker::TopicFrequencyChecker()
{
  ros::NodeHandle nh;

  // Load topics from parameter server
  XmlRpc::XmlRpcValue topic_frequency_list;
  if (!nh.getParam("topic_frequency_analyzer", topic_frequency_list))
    ROS_ERROR("[TopicFrequencyAnalyzer] Could not get \"topic_frequency_analyzer\" from param server");

  if (topic_frequency_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("[TopicFrequencyAnalyzer] Parameter topic_frequency_analyzer must be a list of dicts");

  for (std::size_t i = 0; i < topic_frequency_list.size(); i++)
  {
    const XmlRpc::XmlRpcValue& dict = topic_frequency_list[i];

    if (dict.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      ROS_ERROR("[TopicFrequencyAnalyzer] Parameter topic_frequency_analyzer must be a list of dicts");

    // check for required nodes
    if (!(dict.hasMember("topic") && dict.hasMember("min_hz") && dict.hasMember("max_hz")))
      ROS_ERROR("[TopicFrequencyAnalyzer] Either topic_name, min_hz or max_hz was not specified");

    // get topic
    if (!(dict["topic"].getType() == XmlRpc::XmlRpcValue::TypeString))
      ROS_ERROR("[TopicFrequencyAnalyzer] topic name was not specified");
    std::string topic_name = static_cast<std::string>(dict["topic"]);
    if (topic_name[0] != '/')
      topic_name = '/' + topic_name;

    // get min max frequency
    double min_freq;
    if (dict["min_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      min_freq = static_cast<double>(dict["min_hz"]);
    else if (dict["min_hz"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      min_freq = static_cast<double>(static_cast<int>(dict["min_hz"]));
    else
      ROS_ERROR("[TopicFrequencyAnalyzer] min_hz must be of type int or double");
    double max_freq;
    if (dict["max_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      max_freq = static_cast<double>(dict["max_hz"]);
    else if (dict["max_hz"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      max_freq = static_cast<double>(static_cast<int>(dict["max_hz"]));
    else
      ROS_ERROR("[TopicFrequencyAnalyzer] max_hz must be of type int or double");

    // Sanity checks
    if (min_freq > max_freq)
      ROS_WARN("[TopicFrequencyAnalyzer] max_hz cannot be less than min_hz");
    if (min_freq < 0 || max_freq < 0)
      ROS_WARN("[TopicFrequencyAnalyzer] Frequencies cannot be negative");

    double timeout = 1.0;
    if (dict.hasMember("timeout"))
    {
      if (dict["timeout"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        timeout = static_cast<double>(dict["timeout"]);
      else if (dict["timeout"].getType() == XmlRpc::XmlRpcValue::TypeInt)
        timeout = static_cast<int>(dict["timeout"]);
      else
        ROS_WARN("[TopicFrequencyAnalyzer] %s: timeout must be of type int or double. Using default value 1 s",
                 topic_name.c_str());
    }

    // Store TimeFrequency object
    TopicData time_frequency(min_freq, max_freq, timeout);
    topics_.insert(std::pair<std::string, TopicData>(topic_name, time_frequency));
    topics_.at(topic_name);

    // Setup subscriber on topic
    ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>(
        topic_name, 100, boost::bind(&TopicFrequencyChecker::topicCallback, this, _1, topic_name));
    topic_subs_.push_back(sub);
  }

  // Print loaded params
  std::stringstream ss;
  for (auto const& topic : topics_)
    ss << std::endl
       << "> " << topic.first << " | min: " << topic.second.min_frequency_required
       << ", max: " << topic.second.max_frequency_required << ", timeout: " << topic.second.timeout;
  ROS_INFO_STREAM("[TopicFrequencyChecker] Watching following topic-frequency paires:" + ss.str());

  frequency_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

  // Set up a timer that triggers timerCallback() at a fixed rate
  calc_timer_ = nh.createTimer(ros::Duration(1.0), &TopicFrequencyChecker::timerCallback, this);
}

void TopicFrequencyChecker::topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic)
{
  ros::Time stamp = ros::Time::now();

  topics_.at(topic).count++;
  topics_.at(topic).new_msgs_received = true;

  if (topics_.at(topic).count < 2)
  {
    topics_.at(topic).last_msg_received = stamp;
    return;
  }

  ros::Duration interval = stamp - topics_.at(topic).last_msg_received;  // TODO replace stamp with msg->header.stamp
  topics_.at(topic).intervals_acc(interval.toSec());
  topics_.at(topic).last_msg_received = stamp;
}

void TopicFrequencyChecker::timerCallback(const ros::TimerEvent& event)
{
  diagnostic_msgs::DiagnosticArray diag_array;

  for (auto& topic : topics_)
  {
    diagnostic_msgs::DiagnosticStatus diag_status;
    diag_status.name = "topic_frequency::" + topic.first;

    // Get data
    bool initialized = topic.second.count > 1;
    if (!initialized)
    {
      diag_status.message = "No messages received";
      diag_status.level = diagnostic_msgs::DiagnosticStatus::STALE;
      diag_array.status.push_back(diag_status);
      continue;
    }

    // Compare actual to desired values
    double avg = 1.0 / ba::rolling_mean(topic.second.intervals_acc);
    bool level_ok;
    if (avg < topic.second.min_frequency_required || avg > topic.second.max_frequency_required)
    {
      diag_status.message = "Is: " + std::to_string(avg) +
                            " Hz, should be: " + std::to_string(topic.second.min_frequency_required) + "-" +
                            std::to_string(topic.second.max_frequency_required) + " Hz";
      diag_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    else
    {
      diag_status.message = "OK";
      diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    }

    // Check if stale
    if (!topic.second.new_msgs_received)
    {
      double dt = (event.current_expected - topic.second.last_msg_received).toSec();
      if (dt > topic.second.timeout)
      {
        diag_status.message = "Time since last new message: " + std::to_string(dt) + " s";
        diag_status.level = diagnostic_msgs::DiagnosticStatus::STALE;
      }
    }

    // Fill status message with key values
    diagnostic_msgs::KeyValue keyval_avg;
    keyval_avg.key = "avg";
    keyval_avg.value = topic.second.new_msgs_received ?
                           std::to_string(1.0 / ba::rolling_mean(topic.second.intervals_acc)) :
                           "no new messages";
    diag_status.values.push_back(keyval_avg);
    topic.second.new_msgs_received = false;

    diagnostic_msgs::KeyValue keyval_min;
    keyval_min.key = "min interval";
    keyval_min.value = std::to_string(ba::min(topic.second.intervals_acc)) + 's';
    diag_status.values.push_back(keyval_min);

    diagnostic_msgs::KeyValue keyval_max;
    keyval_max.key = "max interval";
    keyval_max.value = std::to_string(ba::max(topic.second.intervals_acc)) + 's';
    diag_status.values.push_back(keyval_max);

    diagnostic_msgs::KeyValue keyval_count;
    keyval_count.key = "message count";
    keyval_count.value = std::to_string(topic.second.count);
    diag_status.values.push_back(keyval_count);

    diag_array.status.push_back(diag_status);
  }

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
