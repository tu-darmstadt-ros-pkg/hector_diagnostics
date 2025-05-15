#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <memory>
#include <algorithm>

class ActiveNodesChecker : public rclcpp::Node
{
public:
  ActiveNodesChecker() : Node("active_nodes_checker")
  {
    diagnostics_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    // Declare and get parameters
    this->declare_parameter<std::vector<std::string>>("active_nodes", std::vector<std::string>());
    std::vector<std::string> nodes_to_be_checked;

    // Add a small delay to allow parameters to be set externally, if necessary.
    // In a real scenario, consider using parameter services or waiting for parameter events.
    RCLCPP_INFO(this->get_logger(), "Waiting briefly for parameters...");
    rclcpp::Rate param_wait_rate(2.0); // Check twice a second
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) { // Wait up to 2.5 seconds
        if (this->get_parameter("active_nodes", nodes_to_be_checked)) {
            if (!nodes_to_be_checked.empty()) {
               break;
            }
        }
        param_wait_rate.sleep();
    }

    if (nodes_to_be_checked.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "[ActiveNodesChecker] Could not get non-empty \"active_nodes\" parameter.");
      rclcpp::shutdown();
      return;
    }

    nodes_to_be_checked_ = nodes_to_be_checked;

    // Print loaded params
    std::stringstream ss;
    for (const auto& node_name : nodes_to_be_checked_)
      ss << std::endl << "> " << node_name;
    RCLCPP_INFO(this->get_logger(), "[ActiveNodesChecker] Checking following nodes:%s", ss.str().c_str());

    // Create timer that triggers at 1Hz
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ActiveNodesChecker::check_nodes, this));
  }

private:
  void check_nodes()
  {
    std::vector<std::string> active_nodes = this->get_node_names();

    // get_node_names() returns fully qualified names like /namespace/node_name
    // We might need to adjust the comparison logic depending on how names are provided in the parameter.
    // This implementation assumes the parameter provides names *without* a leading '/'
    // and compares against the base name part of the active nodes.

    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    diagnostic_array.header.stamp = this->get_clock()->now();

    for (const std::string& target_node_base_name : nodes_to_be_checked_)
    {
      diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
      std::string target_node_check_name = target_node_base_name;

      // Construct the name for the diagnostic status, prefixing with '/' if not present for consistency
      if (target_node_check_name.empty() || target_node_check_name[0] != '/')
      {
         target_node_check_name = '/' + target_node_check_name;
      }
      diagnostic_status.name = "active_nodes" + target_node_check_name; // e.g., active_nodes/my_node

      bool found = false;
      for (const std::string& active_node_fqn : active_nodes) {
          // Basic check: see if the fully qualified active node name ends with the target name (prefixed with /)
          if (active_node_fqn.length() >= target_node_check_name.length() &&
              active_node_fqn.substr(active_node_fqn.length() - target_node_check_name.length()) == target_node_check_name) {
              found = true;
              break;
          }
          // Also check if the base name matches exactly (handles cases where param has / but node name doesn't somehow, though unlikely)
          if (active_node_fqn == target_node_check_name) {
              found = true;
              break;
          }
      }

      if (found)
      {
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diagnostic_status.message = "OK";
      }
      else
      {
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diagnostic_status.message = "Not active";
      }

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "node";
      // Store the name exactly as provided in the parameter for clarity
      kv.value = target_node_base_name;
      diagnostic_status.values.push_back(kv);

      diagnostic_array.status.push_back(diagnostic_status);
    }

    diagnostics_publisher_->publish(diagnostic_array);
  }

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> nodes_to_be_checked_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActiveNodesChecker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
