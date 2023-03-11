#include <hector_software_monitor/analyzers/simple_analyzer.h>

namespace hector_software_monitor
{
bool SimpleAnalyzer::init(const std::string base_path, const ros::NodeHandle& nh)
{
  timeout_ = nh.param("timeout", 5.0);

  if (!nh.getParam("path", nice_name_))
  {
    ROS_ERROR("[SimpleAnalyzer] Parameters not given at path \"path\". Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (base_path == "/")
    path_ = nice_name_;
  else
    path_ = base_path + "/" + nice_name_;

  if (path_.find('/') != 0)
    path_ = "/" + path_;

  if (!nh.getParam("find_prefix", prefix_))
  {
    ROS_ERROR("[SimpleAnalyzer] Need parameter 'find_prefix' to determine which diagnostic messages should be matched. "
              "Name: %s, namespace: %s",
              nice_name_.c_str(), nh.getNamespace().c_str());
    return false;
  }

  return true;
}

bool SimpleAnalyzer::match(const std::string name)
{
  return name.find(prefix_) == 0;
}

bool SimpleAnalyzer::analyze(const boost::shared_ptr<diagnostic_aggregator::StatusItem> item)
{
  items_[item->getName()] = item;
  return true;
}

std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> SimpleAnalyzer::report()
{
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> header_status(new diagnostic_msgs::DiagnosticStatus());
  header_status->name = path_;
  header_status->level = diagnostic_aggregator::DiagnosticLevel::Level_OK;
  header_status->message = "OK";

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus>> processed;

  processed.push_back(header_status);

  bool all_stale = true;

  for (const std::pair<const std::string, boost::shared_ptr<diagnostic_aggregator::StatusItem>>& p : items_)
  {
    std::string name_without_prefix = p.first;
    boost::shared_ptr<diagnostic_aggregator::StatusItem> item = p.second;

    // Remove prefix
    if (name_without_prefix.find(prefix_) != 0)
      ROS_ERROR("[SimpleAnalyzer] Cannot remove prefix: Prefix not found. name: %s, prefix: %s",
                name_without_prefix.c_str(), prefix_.c_str());
    name_without_prefix.replace(0, prefix_.size(), "");
    if (name_without_prefix[0] == '/')
      name_without_prefix.replace(0, 1, "");

    // The rqt_robot_monitor uses slashes in the name of a diagnostic_agg as indication to split into subtrees. Thus, we
    // cannot use a name like 'joints/joint_states'. As a workaround, replace all slashes with space as it is done in
    // generic_analyzer.cpp
    std::string name = path_ + "/" + diagnostic_aggregator::getOutputName(name_without_prefix);

    bool stale = (ros::Time::now() - item->getLastUpdateTime()).toSec() > timeout_;

    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> new_status = item->toStatusMsg(path_, stale);
    new_status->name = name;

    /*
     * Analyze the item here and set level, message etc. according to your conditions
     * e.g.
     * if (cond)
     * {
     *   new_status->level = ...
     * }
     */

    const int8_t level = new_status->level;
    header_status->level = std::max(header_status->level, level);

    diagnostic_msgs::KeyValue kv;
    kv.key = name_without_prefix;
    kv.value = new_status->message;
    header_status->values.push_back(kv);

    all_stale = all_stale && stale;

    processed.push_back(new_status);
  }

  // Header is not stale unless all subs are
  if (all_stale)
    header_status->level = diagnostic_aggregator::DiagnosticLevel::Level_Stale;
  else if (header_status->level == int(diagnostic_aggregator::DiagnosticLevel::Level_Stale))
    header_status->level = diagnostic_aggregator::DiagnosticLevel::Level_Error;

  header_status->message = diagnostic_aggregator::valToMsg(header_status->level);

  return processed;
}
}  // namespace hector_software_monitor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_software_monitor::SimpleAnalyzer, diagnostic_aggregator::Analyzer)
