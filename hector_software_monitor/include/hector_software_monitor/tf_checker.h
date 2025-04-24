#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <vector>
#include <string>
#include <memory> // For std::unique_ptr

namespace hector_software_monitor
{

/**
 * @brief Checks the availability and timeliness of specified TF transforms and publishes diagnostics.
 */
class TFChecker : public rclcpp::Node // Inherit from rclcpp::Node
{
public:
  /**
   * @brief Stores information about a required transform.
   */
  struct RequiredTransform
  {
    RequiredTransform(std::string source_frame, std::string target_frame, double timeout_sec);
    ~RequiredTransform() = default;

    std::string source_frame;
    std::string target_frame;
    rclcpp::Duration timeout; // Use rclcpp::Duration
  };

  /**
   * @brief Constructor for the TFChecker node.
   */
  TFChecker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TFChecker() override = default; // Use override

private:
  /**
   * @brief Called periodically to check transforms and publish diagnostics.
   */
  void timerCallback(); // No event argument needed

  // Parameter handling
  void declareParameters();
  void loadParameters();

  std::vector<RequiredTransform> transforms_;

  // TF2 members
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // Use shared_ptr
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // Use shared_ptr

  // ROS 2 members
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

} // namespace hector_software_monitor
