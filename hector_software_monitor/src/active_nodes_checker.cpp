#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "active_nodes_checker");

  ros::NodeHandle nh;

  ros::Publisher diagnostics_publisher = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100);

  // Wait for the parameters to be loaded
  ros::Duration(0.5).sleep();
  std::vector<std::string> nodes_to_be_checked;
  if (!nh.getParam("active_nodes", nodes_to_be_checked))
  {
    ROS_ERROR("[ActiveNodesChecker] Could not get nodes to be checked \"active_nodes\" from param server");
    return 0;
  }

  // Print loaded params
  std::stringstream ss;
  for (auto const& node : nodes_to_be_checked)
    ss << std::endl << "> " << node;
  ROS_INFO_STREAM("[ActiveNodesChecker] Checking following nodes:" + ss.str());

  while (ros::ok())
  {
    std::vector<std::string> active_nodes;
    if (!ros::master::getNodes(active_nodes))
    {
      ROS_ERROR("[ActiveNodesChecker] Could not retrieve active nodes with ros::master::getNodes");
    }

    for (std::string node_name : nodes_to_be_checked)
    {
      diagnostic_msgs::DiagnosticStatus diagnostic_status;

      if (node_name[0] != '/')
      {
        node_name = '/' + node_name;
      }

      diagnostic_status.name = "active_nodes" + node_name;

      if (std::find(active_nodes.begin(), active_nodes.end(), node_name) != active_nodes.end())
      {
        diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::OK;
        diagnostic_status.message = "OK";
      }
      else
      {
        diagnostic_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        diagnostic_status.message = "Not active";
      }
      diagnostic_msgs::DiagnosticArray diagnostic_array;
      diagnostic_array.header.stamp = ros::Time::now();
      diagnostic_array.status.push_back(diagnostic_status);

      diagnostics_publisher.publish(diagnostic_array);
    }
    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  return 0;
}
