#include <hector_software_monitor/analyzers/topic_frequency_analyzer.h>

namespace hector_software_monitor
{
TopicFrequencyAnalyzer::TopicFrequency::TopicFrequency(double min, double max, double stale_timeout)
  : last_update(ros::Time::now())
  , avg(0.0)
  , min_frequency_required(min)
  , max_frequency_required(max)
  , stale_timeout(stale_timeout)
{
}

bool TopicFrequencyAnalyzer::init(const std::string base_path, const ros::NodeHandle& nh)
{
  // General setup
  timeout_ = nh.param("timeout", 5.0);

  std::string nice_name;
  if (!nh.getParam("path", nice_name))
  {
    ROS_ERROR("[TopicFrequencyAnalyzer] Parameters not given at path \"path\". Namespace: %s",
              nh.getNamespace().c_str());
    return false;
  }
  nice_name_ = nice_name;
  if (base_path == "/")
    path_ = nice_name;

  else
    path_ = ros::names::append(base_path, nice_name);

  if (path_.find('/') != 0)
    path_ = ros::names::append("/", path_);

  if (!nh.getParam("name", match_name_))
  {
    ROS_ERROR("[TopicFrequencyAnalyzer] Need parameter 'name' to determine which diagnostic messages should be "
              "matched. Name: %s, namespace: %s",
              nice_name.c_str(), nh.getNamespace().c_str());
    return false;
  }

  // Setup of TopicFrequencyAnalyzer features

  ros::NodeHandle global_nh;

  XmlRpc::XmlRpcValue topic_frequency_list;
  if (!global_nh.getParam("topic_frequency_analyzer", topic_frequency_list))
  {
    ROS_ERROR("[TopicFrequencyAnalyzer] Could not get \"topic_frequency_analyzer\" from param server");
    return false;
  }
  if (topic_frequency_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("[TopicFrequencyAnalyzer] Parameter topic_frequency_analyzer must be a list of dicts");
    return false;
  }

  // NOLINTNEXTLINE
  for (std::size_t i = 0; i < topic_frequency_list.size(); i++)
  {
    const XmlRpc::XmlRpcValue& dict = topic_frequency_list[i];

    if (dict.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("[TopicFrequencyAnalyzer] Parameter topic_frequency_analyzer must be a list of dicts");
      return false;
    }

    if (!(dict.hasMember("topic") && dict.hasMember("min_hz") && dict.hasMember("max_hz")))
    {
      ROS_ERROR("[TopicFrequencyAnalyzer] Either topic_name, min_hz or max_hz was not specified");
      return false;
    }

    if (!(dict["topic"].getType() == XmlRpc::XmlRpcValue::TypeString))
    {
      ROS_ERROR("[TopicFrequencyAnalyzer] topic name was not specified");
      return false;
    }
    std::string topic_name = static_cast<std::string>(dict["topic"]);
    if (topic_name[0] != '/')
      topic_name = '/' + topic_name;

    double min_freq;
    if (dict["min_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      min_freq = static_cast<double>(dict["min_hz"]);
    }
    else if (dict["min_hz"].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      min_freq = static_cast<double>(static_cast<int>(dict["min_hz"]));
    }
    else
    {
      ROS_ERROR("[TopicFrequencyAnalyzer] min_hz must be of type int or double");
      return false;
    }

    double max_freq;
    if (dict["max_hz"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      max_freq = static_cast<double>(dict["max_hz"]);
    }
    else if (dict["max_hz"].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      max_freq = static_cast<double>(static_cast<int>(dict["max_hz"]));
    }
    else
    {
      ROS_ERROR("[TopicFrequencyAnalyzer] max_hz must be of type int or double");
      return false;
    }

    // Sanity checks
    if (min_freq > max_freq)
      ROS_WARN("[TopicFrequencyAnalyzer] max_hz cannot be less than min_hz");
    if (min_freq < 0 || max_freq < 0)
      ROS_WARN("[TopicFrequencyAnalyzer] Frequencies cannot be negative");

    double topic_timeout = 1.0;

    if (dict.hasMember("stale_timeout"))
    {
      if (dict["stale_timeout"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {
        topic_timeout = static_cast<double>(dict["stale_timeout"]);
      }
      else if (dict["stale_timeout"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        topic_timeout = static_cast<int>(dict["stale_timeout"]);
      }
      else
      {
        ROS_WARN("[TopicFrequencyAnalyzer] %s: stale_timeout must be of type int or double. Using default value 1 s",
                 topic_name.c_str());
      }
    }

    // Save
    TopicFrequency topic_frequency(min_freq, max_freq, topic_timeout);
    topics_.insert(std::pair<std::string, TopicFrequency>(topic_name, topic_frequency));
  }

  // Print loaded params
  std::stringstream ss;
  for (auto const& topic : topics_)
    ss << std::endl
       << "> " << topic.first << " | min " << topic.second.min_frequency_required << ", max "
       << topic.second.max_frequency_required << ", timeout " << topic.second.stale_timeout;
  ROS_INFO_STREAM("[TopicFrequencyAnalyzer] Watching following topic-frequency paires:" + ss.str());

  return true;
}

bool TopicFrequencyAnalyzer::match(const std::string name)
{
  return name == match_name_;
}

bool TopicFrequencyAnalyzer::analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item)
{
  item_ = item;
  last_analyzed_msg_ = ros::Time::now();
  return true;
}

std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> TopicFrequencyAnalyzer::report()
{
  ros::Time report_stamp = ros::Time::now();

  // skip report if no item has been received yet
  if (!item_)
    return std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>>();

  bool analyzed_msg_stale = (ros::Time::now().toSec() - last_analyzed_msg_.toSec()) > timeout_;

  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> header_status(new diagnostic_msgs::DiagnosticStatus());
  header_status->level = diagnostic_aggregator::DiagnosticLevel::Level_OK;
  header_status->name = path_;

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> processed;

  if (analyzed_msg_stale)
  {
    header_status->level = diagnostic_aggregator::DiagnosticLevel::Level_Stale;
    header_status->message = "All stale. Is node frequency_publisher running?";

    processed.push_back(header_status);
    return processed;
  }

  bool all_stale = true;

  // loop over all topics
  for (auto& topic : topics_)
  {
    std::string new_avg_str =
        item_->getValue(topic.first + "_avg");  // topic.first: name, topic.second: TopicFrequency object
    if (new_avg_str.empty())
      continue;

    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> new_status =
        boost::make_shared<diagnostic_msgs::DiagnosticStatus>();
    if (new_avg_str != "no new messages")
    {
      topic.second.avg = std::stod(new_avg_str);
      topic.second.last_update = report_stamp;
    }

    // Compare
    if (topic.second.avg < topic.second.min_frequency_required ||
        topic.second.avg > topic.second.max_frequency_required)
    {
      new_status->message = "Is: " + std::to_string(topic.second.avg) +
                            " Hz, should be: " + std::to_string(topic.second.min_frequency_required) + "-" +
                            std::to_string(topic.second.max_frequency_required) + " Hz";
      new_status->level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    else
    {
      new_status->message = "OK";
      new_status->level = diagnostic_msgs::DiagnosticStatus::OK;
    }

    // Check if stale
    bool current_topic_stale = (report_stamp - topic.second.last_update).toSec() > topic.second.stale_timeout;
    if (current_topic_stale)
    {
      new_status->message = "No new messages received";
      new_status->level = diagnostic_msgs::DiagnosticStatus::STALE;
    }

    // Fill message with values
    diagnostic_msgs::KeyValue keyval_avg;
    diagnostic_msgs::KeyValue keyval_min;
    diagnostic_msgs::KeyValue keyval_max;
    diagnostic_msgs::KeyValue keyval_count;

    keyval_avg.key = "average rate";
    keyval_avg.value = std::to_string(topic.second.avg);
    keyval_min.key = "min interval";
    keyval_min.value = item_->getValue(topic.first + "_min");
    keyval_max.key = "max interval";
    keyval_max.value = item_->getValue(topic.first + "_max");
    keyval_count.key = "total messages received";
    keyval_count.value = item_->getValue(topic.first + "_count");

    new_status->values.push_back(keyval_avg);
    new_status->values.push_back(keyval_min);
    new_status->values.push_back(keyval_max);
    new_status->values.push_back(keyval_count);

    all_stale = all_stale && current_topic_stale;

    // Update the header status level: Header is only stale if all children are stale
    header_status->level = std::max(header_status->level, new_status->level);

    // Add path to status name
    std::string topic_name = diagnostic_aggregator::getOutputName(topic.first);
    new_status->name = ros::names::append(path_, topic_name);
    processed.push_back(new_status);

    // Header is not stale unless all subs are
    if (all_stale)
      header_status->level = diagnostic_aggregator::DiagnosticLevel::Level_Stale;
    else if (header_status->level == int(diagnostic_aggregator::DiagnosticLevel::Level_Stale))
      header_status->level = diagnostic_aggregator::DiagnosticLevel::Level_Error;
  }

  // Set the status level for the header: stale or error if any msg is stale or error
  header_status->message = diagnostic_aggregator::valToMsg(header_status->level);
  processed.push_back(header_status);

  return processed;
}
}  // namespace hector_software_monitor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_software_monitor::TopicFrequencyAnalyzer, diagnostic_aggregator::Analyzer)
