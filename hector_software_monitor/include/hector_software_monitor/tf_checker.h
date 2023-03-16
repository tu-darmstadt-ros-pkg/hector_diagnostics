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
  /**
   * @brief The RequiredTransform class stores information about the transform that should be checked
   */
  struct RequiredTransform
  {
    RequiredTransform(std::string source_frame, std::string target_frame, double timeout);
    ~RequiredTransform() = default;

    std::string source_frame;
    std::string target_frame;

    /**
     * @brief timeout After this duration without receiving a new message, the transform is marked as stale
     */
    ros::Duration timeout;

    /**
     * @brief is_static indicates whether a topic was published on tf_static
     */
    bool is_static;
  };

  TFChecker();
  ~TFChecker() = default;

private:
  /**
 ´  * @brief timerCallback A function that is called periodically to analyze all given transforms and publish their
   * availability as diagnostic_msgs
   */
  void timerCallback(const ros::TimerEvent& event);

  /**
   * @brief TFStaticCallback  A callback method for topic tf_static to determine which of the required transforms are
   * static and thus not to be checked for time out
   */
  void TFStaticCallback(const tf2_msgs::TFMessageConstPtr& msg);

  std::vector<RequiredTransform> transforms_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Timer publish_timer_;
  ros::Publisher diagnostics_pub_;
  ros::Subscriber tf_static_sub_;
};

}  // namespace hector_software_monitor
