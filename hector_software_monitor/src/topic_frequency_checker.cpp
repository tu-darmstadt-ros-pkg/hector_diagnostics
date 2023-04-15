#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <hector_software_monitor/topic_frequency_checker.h>

namespace hector_software_monitor
{
TopicFrequencyChecker::Connection::Connection()
  : frequency(0.0), last_msg_received(ros::Time(0.0)), delivered_msgs(0), dropped_msgs(0)
{
}

TopicFrequencyChecker::TopicData::TopicData(double min, double max, double timeout)
  : min_frequency_required(min), max_frequency_required(max), timeout(timeout), initialized(false)
{
}

TopicFrequencyChecker::TopicFrequencyChecker()
{
  ros::NodeHandle nh;

  // Check if statistics are enabled
  if (!nh.param("/enable_statistics", false))
    ROS_ERROR("[TopicFrequencyChecker] Parameter \'/enable_statistics\' must be set for this node to work");

  // Load topics from parameter server
  XmlRpc::XmlRpcValue topic_frequency_list;
  if (!nh.getParam("topic_frequency_analyzer", topic_frequency_list))
    ROS_ERROR("[TopicFrequencyChecker] Could not get \"topic_frequency_analyzer\" from param server");

  if (topic_frequency_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("[TopicFrequencyChecker] Parameter topic_frequency_analyzer must be a list of dicts");

  for (std::size_t i = 0; i < topic_frequency_list.size(); i++)
  {
    const XmlRpc::XmlRpcValue& dict = topic_frequency_list[i];

    if (dict.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      ROS_ERROR("[TopicFrequencyChecker] Parameter topic_frequency_analyzer must be a list of dicts");

    // check for required nodes
    if (!(dict.hasMember("topic") && dict.hasMember("min_hz") && dict.hasMember("max_hz")))
      ROS_ERROR("[TopicFrequencyChecker] Either topic_name, min_hz or max_hz was not specified");

    // get topic
    if (!(dict["topic"].getType() == XmlRpc::XmlRpcValue::TypeString))
      ROS_ERROR("[TopicFrequencyChecker] topic name was not specified");
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
      ROS_ERROR("[TopicFrequencyChecker] min_hz must be of type int or double");
    double max_freq;
    if (dict["max_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      max_freq = static_cast<double>(dict["max_hz"]);
    else if (dict["max_hz"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      max_freq = static_cast<double>(static_cast<int>(dict["max_hz"]));
    else
      ROS_ERROR("[TopicFrequencyChecker] max_hz must be of type int or double");

    // Sanity checks
    if (min_freq > max_freq)
      ROS_WARN("[TopicFrequencyChecker] max_hz cannot be less than min_hz");
    if (min_freq < 0 || max_freq < 0)
      ROS_WARN("[TopicFrequencyChecker] Frequencies cannot be negative");

    double timeout = 3.0;
    if (dict.hasMember("timeout"))
    {
      if (dict["timeout"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        timeout = static_cast<double>(dict["timeout"]);
      else if (dict["timeout"].getType() == XmlRpc::XmlRpcValue::TypeInt)
        timeout = static_cast<int>(dict["timeout"]);
      else
        ROS_WARN("[TopicFrequencyChecker] %s: timeout must be of type int or double. Using default value 1 s",
                 topic_name.c_str());
    }

    // Store TimeFrequency object
    TopicData time_frequency(min_freq, max_freq, timeout);
    topics_.insert(std::pair<std::string, TopicData>(topic_name, time_frequency));
    topics_.at(topic_name);
  }

  // Print loaded params
  std::stringstream ss;
  for (auto const& topic : topics_)
    ss << std::endl
       << "> " << topic.first << " | min: " << topic.second.min_frequency_required
       << ", max: " << topic.second.max_frequency_required << ", timeout: " << topic.second.timeout;
  ROS_INFO_STREAM("[TopicFrequencyChecker] Watching following topic-frequency paires:" + ss.str());

  diagnostics_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

  // Setup subscriber for statistics topic
  stats_sub_ =
      nh.subscribe<rosgraph_msgs::TopicStatistics>("/statistics", 10, &TopicFrequencyChecker::statCallback, this);
  timer_ = nh.createTimer(ros::Duration(1.0), &TopicFrequencyChecker::timerCallback, this);
  timer_.start();
}

void TopicFrequencyChecker::statCallback(const rosgraph_msgs::TopicStatisticsConstPtr& msg)
{
  ros::Time stamp = ros::Time::now();

  // Skip unwanted statistics
  auto topic_it = topics_.find(msg->topic);
  if (topic_it == topics_.end())
    return;

  // Insert connection in map
  auto con_key = std::pair<std::string, std::string>(msg->node_pub, msg->node_sub);
  if (topic_it->second.connections.find(con_key) == topic_it->second.connections.end())
    topic_it->second.connections.insert(
        std::pair<std::pair<std::string, std::string>, Connection>(con_key, Connection()));

  topic_it->second.connections.at(con_key).frequency = 1.0 / msg->period_mean.toSec();
  topic_it->second.connections.at(con_key).last_msg_received = stamp;

  topic_it->second.initialized = true;  // At least one connection has been established
}

void TopicFrequencyChecker::timerCallback(const ros::TimerEvent& event)
{
  // Get list of all active nodes to later remove the connections without active publisher/subscriber
  std::vector<std::string> active_nodes;
  ros::master::getNodes(active_nodes);

  diagnostic_msgs::DiagnosticArray diag_array;

  for (auto& topic : topics_)
  {
    diagnostic_msgs::DiagnosticStatus diag_status;
    diag_status.name = "topic_frequency::" + topic.first;

    // Erase all connections without active publisher and subscriber
    for (auto it = topic.second.connections.cbegin(); it != topic.second.connections.cend();)
    {
      // Check pubsliher
      bool connection_active =
          std::find(active_nodes.begin(), active_nodes.end(), it->first.first) != active_nodes.end();
      // Check subscriber
      connection_active &= std::find(active_nodes.begin(), active_nodes.end(), it->first.second) != active_nodes.end();

      if (!connection_active)
      {
        topic.second.connections.erase(it++);
      }
      else
      {
        ++it;
      }
    }

    size_t error = 0;
    size_t stale = 0;
    double error_frequency;

    for (auto& connection : topic.second.connections)
    {
      auto frequency = connection.second.frequency;
      if (event.current_expected.toSec() - connection.second.last_msg_received.toSec() > topic.second.timeout)
      {
        stale++;
      }
      else if (frequency < topic.second.min_frequency_required || frequency > topic.second.max_frequency_required)
      {
        error++;
        error_frequency = frequency;
      }

      diagnostic_msgs::KeyValue keyval_frequency;
      keyval_frequency.key = connection.first.first + " ->" + connection.first.second;  // pub/sub
      keyval_frequency.value = std::to_string(frequency);
      diag_status.values.push_back(keyval_frequency);

      diag_array.status.push_back(diag_status);
    }

    if (stale > 0)
    {
      diag_status.message =
          std::to_string(stale) + "/" + std::to_string(topic.second.connections.size()) + " connections are stale";
      diag_status.level = diagnostic_msgs::DiagnosticStatus::STALE;
    }
    if (error > 0)  // Error has higher priority than stale
    {
      diag_status.message = "Is: " + std::to_string(error_frequency) +
                            " Hz, should be: " + std::to_string(topic.second.min_frequency_required) + "-" +
                            std::to_string(topic.second.max_frequency_required) + " Hz";
      diag_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    if (!stale && !error)
    {
      diag_status.message = "OK";
      diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    }

    // If there are no connections for this topic
    if (topic.second.connections.empty())
    {
      diag_status.message =
          topic.second.initialized ? "All publishers or subscribers shut down" : "No message received yet";
      diag_status.level = diagnostic_msgs::DiagnosticStatus::STALE;
    }

    diag_array.status.push_back(diag_status);
  }

  diag_array.header.stamp = event.current_expected;

  diagnostics_pub_.publish(diag_array);
}
}  // namespace hector_software_monitor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "topic_frequency_checker");
  hector_software_monitor::TopicFrequencyChecker topic_frequency_checker;
  ros::spin();
  return 0;
}
