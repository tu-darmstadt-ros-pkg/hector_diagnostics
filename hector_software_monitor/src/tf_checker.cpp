#include <hector_software_monitor/tf_checker.h>

namespace hector_software_monitor
{
TFChecker::RequiredTransform::RequiredTransform(std::string source_frame, std::string target_frame, double timeout)
  : source_frame(source_frame), target_frame(target_frame), timeout(timeout)
{
}

TFChecker::TFChecker() : tf_listener_(tf_buffer_)
{
  ros::NodeHandle nh;

  // Load topics from parameter server
  XmlRpc::XmlRpcValue transform_list;
  if (!nh.getParam("tf_transforms", transform_list))
  {
    ROS_ERROR("[TFChecker] Could not get \"tf_transforms\" from param server");
  }
  if (transform_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("[TFChecker] Parameter tf_transforms must be a list of dicts");
  }

  // Iterate over all given transforms
  for (std::size_t i = 0; i < transform_list.size(); i++)
  {
    const XmlRpc::XmlRpcValue& dict = transform_list[i];

    if (dict.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("[TFChecker] Parameter \'tf_transforms\' must be a list of dicts");
    }

    if (!(dict.hasMember("source_frame") && dict["source_frame"].getType() == XmlRpc::XmlRpcValue::TypeString &&
          dict.hasMember("target_frame") && dict["target_frame"].getType() == XmlRpc::XmlRpcValue::TypeString))
    {
      ROS_ERROR("[TFChecker] Parameter tf_transforms required fields \'source_frame\', \'target_frame\' of type "
                "string");
    }

    if (dict.hasMember("timeout") && !(dict["timeout"].getType() == XmlRpc::XmlRpcValue::TypeInt &&
                                       dict["timeout"].getType() != XmlRpc::XmlRpcValue::TypeDouble))
    {
      ROS_ERROR("[TFChecker] Parameter tf_transforms required field \'timeout\' of type int or double");
    }

    // Save transform in vector
    std::string source_frame = static_cast<std::string>(dict["source_frame"]);
    std::string target_frame = static_cast<std::string>(dict["target_frame"]);
    double timeout = 1.0;
    if (dict.hasMember("timeout"))
    {
      if (dict["timeout"].getType() == XmlRpc::XmlRpcValue::TypeInt)
        timeout = static_cast<int>(dict["timeout"]);
      else if (dict["timeout"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        timeout = static_cast<double>(dict["timeout"]);
    }

    transforms_.push_back(RequiredTransform(source_frame, target_frame, timeout));
  }

  std::stringstream info_stream;
  for (const auto& transform : transforms_)
    info_stream << "> " << transform.source_frame << " -> " << transform.target_frame
                << " | timeout: " << transform.timeout << std::endl;
  ROS_INFO("[TF_Checker] Watching the following tf transforms:\n%s", info_stream.str().c_str());

  // Setup subscriber, publisher and timer
  diagnostics_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
  publish_timer_ = nh.createTimer(ros::Duration(1.0), &TFChecker::timerCallback, this);
}

void TFChecker::timerCallback(const ros::TimerEvent& event)
{
  diagnostic_msgs::DiagnosticArray diag_array;

  for (const auto& transform : transforms_)
  {
    bool status;
    geometry_msgs::TransformStamped tf_msg;
    bool transform_found = false;

    // Check if transform exists
    try
    {
      tf_msg = tf_buffer_.lookupTransform(transform.source_frame, transform.target_frame, ros::Time(0));
      transform_found = true;
    }
    catch (...)
    {
      status = false;
    }

    // If transform exists, check time stamp
    bool is_static = false;
    ros::Duration dt;
    if (transform_found)
    {
      // tf_static msgs always have stamp 0
      if (tf_msg.header.stamp == ros::Time(0))
      {
        is_static = true;
        status = true;
      }
      // for non static msgs check time since last update
      else
      {
        dt = event.current_expected - tf_msg.header.stamp;
        status = dt <= transform.timeout;
      }
    }

    // Fill status message
    diagnostic_msgs::DiagnosticStatus diag_status;
    diag_status.name = "tf::" + transform.source_frame + "->" + transform.target_frame;
    diag_status.level = status ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
    if (status)
      diag_status.message = "OK";
    else if (transform_found)
      diag_status.message = "Last message received " + std::to_string(int(dt.toSec())) + " seconds ago";
    else
      diag_status.message = "Transform not available";
    if (is_static)
      diag_status.message += " (static)";

    diag_array.status.push_back(diag_status);
  }

  // Publish array
  diag_array.header.stamp = event.current_expected;
  diagnostics_pub_.publish(diag_array);
}
}  // namespace hector_software_monitor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_checker");
  hector_software_monitor::TFChecker tf_checker;
  ros::spin();
  return 0;
}
