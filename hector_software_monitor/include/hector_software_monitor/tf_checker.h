#pragma once

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

namespace hector_software_monitor
{
/**
 * @brief TODO
 */
class TFChecker
{
public:
  struct RequiredTransform
  {
    RequiredTransform(std::string source_frame, std::string target_frame, double timeout);
    ~RequiredTransform() = default;

    std::string source_frame;
    std::string target_frame;
    ros::Duration timeout;
  };

  TFChecker();
  ~TFChecker() = default;

private:
  /**
 ´  * @brief timerCallback A function that is called periodically to analyze all given transforms and publish their
   * availability as diagnostic_msgs
   */
  void timerCallback(const ros::TimerEvent& event);

  std::vector<RequiredTransform> transforms_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Timer publish_timer_;
  ros::Publisher diagnostics_pub_;
};

}  // namespace hector_software_monitor
