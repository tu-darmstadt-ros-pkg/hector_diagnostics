#include <hector_software_monitor/tf_checker.h>
#include <tf2/exceptions.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter_map.hpp>

#include <chrono> // For chrono literals
#include <sstream>
#include <vector>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace hector_software_monitor
{

TFChecker::RequiredTransform::RequiredTransform(std::string source_frame, std::string target_frame, double timeout_sec)
  : source_frame(std::move(source_frame)), target_frame(std::move(target_frame)), timeout(rclcpp::Duration::from_seconds(timeout_sec))
{
}

TFChecker::TFChecker(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tf_checker", options) // Pass node name and options to base class
{
  // Initialize TF2 Buffer and Listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Declare and load parameters
  declareParameters();
  loadParameters();

  // Setup publisher and timer
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  publish_timer_ = this->create_wall_timer(1s, std::bind(&TFChecker::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "TFChecker node initialized.");
}

void TFChecker::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor source_desc;
  source_desc.description = "List of source frames for TF checks.";
  this->declare_parameter<std::vector<std::string>>("tf_source_frames", std::vector<std::string>{}, source_desc);

  rcl_interfaces::msg::ParameterDescriptor target_desc;
  target_desc.description = "List of target frames corresponding to tf_source_frames.";
  this->declare_parameter<std::vector<std::string>>("tf_target_frames", std::vector<std::string>{}, target_desc);

  rcl_interfaces::msg::ParameterDescriptor timeout_desc;
  timeout_desc.description = "List of timeouts (seconds) corresponding to tf_source_frames/tf_target_frames.";
  this->declare_parameter<std::vector<double>>("tf_timeouts", std::vector<double>{}, timeout_desc);
}

void TFChecker::loadParameters()
{
  std::vector<std::string> source_frames = this->get_parameter("tf_source_frames").as_string_array();
  std::vector<std::string> target_frames = this->get_parameter("tf_target_frames").as_string_array();
  std::vector<double> timeouts = this->get_parameter("tf_timeouts").as_double_array();

  if (source_frames.size() != target_frames.size() || source_frames.size() != timeouts.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Parameter array sizes mismatch! 'tf_source_frames' (%zu), 'tf_target_frames' (%zu), 'tf_timeouts' (%zu) must have the same number of elements.",
                 source_frames.size(), target_frames.size(), timeouts.size());
    return; // Or throw?
  }

  transforms_.clear();
  std::stringstream info_stream;
  info_stream << "Watching the following tf transforms:\n";
  for (size_t i = 0; i < source_frames.size(); ++i) {
    if (timeouts[i] <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Timeout for %s -> %s is non-positive (%.2f), using default 1.0s",
                    source_frames[i].c_str(), target_frames[i].c_str(), timeouts[i]);
        timeouts[i] = 1.0;
    }
    transforms_.emplace_back(source_frames[i], target_frames[i], timeouts[i]);
    info_stream << "> " << source_frames[i] << " -> " << target_frames[i]
                << " | timeout: " << timeouts[i] << "s\n";
  }

  RCLCPP_INFO(this->get_logger(), info_stream.str().c_str());

  if (transforms_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid TF transforms configured to check.");
  }
}

void TFChecker::timerCallback() // No event argument
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->get_clock()->now();

  for (const auto& transform : transforms_)
  {
    diagnostic_msgs::msg::DiagnosticStatus diag_status;
    diag_status.name = "tf::" + transform.source_frame + "->" + transform.target_frame;

    bool status = false; // Default to error
    geometry_msgs::msg::TransformStamped tf_msg;
    bool transform_found = false;
    bool is_static = false;
    rclcpp::Duration dt = rclcpp::Duration(0, 0);
    std::string error_string;

    try
    {
      // Use tf2::TimePointZero for latest available transform
      tf_msg = tf_buffer_->lookupTransform(transform.target_frame, transform.source_frame, tf2::TimePointZero);
      transform_found = true;

      // Check if static (stamp is zero)
      if (tf_msg.header.stamp == rclcpp::Time(0, 0, this->get_clock()->get_clock_type())) {
        is_static = true;
        status = true; // Static transforms are always considered OK if found
        diag_status.message = "OK (static)";
      }
      else
      {
        // Check timestamp for non-static transforms
        dt = rclcpp::Time(diag_array.header.stamp) - rclcpp::Time(tf_msg.header.stamp);
        if (dt >= rclcpp::Duration(0, 0) && dt <= transform.timeout) {
            status = true;
            diag_status.message = "OK";
        } else if (dt < rclcpp::Duration(0, 0)) {
            status = false;
            diag_status.message = "Transform is from the future!";
            RCLCPP_WARN(this->get_logger(), "Transform %s -> %s has future timestamp! Current: %f, Transform: %f",
                        transform.source_frame.c_str(), transform.target_frame.c_str(),
                        rclcpp::Time(diag_array.header.stamp).seconds(), rclcpp::Time(tf_msg.header.stamp).seconds());
        } else {
            status = false;
            diag_status.message = "Last message received " + std::to_string(rclcpp::Duration(dt).seconds()) + " seconds ago";
        }
      }
    }
    catch (const tf2::TransformException & ex)
    {
      transform_found = false;
      status = false;
      diag_status.message = "Transform not available";
      error_string = ex.what();
      RCLCPP_DEBUG(this->get_logger(), "TF lookup failed for %s -> %s: %s",
                    transform.source_frame.c_str(), transform.target_frame.c_str(), error_string.c_str());
    }

    // Set diagnostic level
    diag_status.level = status ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    // Add key-value pairs
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "source frame";
    kv.value = transform.source_frame;
    diag_status.values.push_back(kv);
    kv.key = "target frame";
    kv.value = transform.target_frame;
    diag_status.values.push_back(kv);
    if (transform_found)
    {
      kv.key = "static";
      kv.value = is_static ? "true" : "false";
      diag_status.values.push_back(kv);
      if (!is_static) {
          kv.key = "last update (sec ago)";
          kv.value = std::to_string(rclcpp::Duration(dt).seconds());
          diag_status.values.push_back(kv);
      }
    } else {
        kv.key = "error";
        kv.value = error_string; // Add TF exception message if lookup failed
        diag_status.values.push_back(kv);
    }

    diag_array.status.push_back(diag_status);
  }

  // Publish array if there are any statuses
  if (!diag_array.status.empty()) {
    diagnostics_pub_->publish(diag_array);
  }
}

} // namespace hector_software_monitor

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // Instantiate the node using make_shared
  auto tf_checker_node = std::make_shared<hector_software_monitor::TFChecker>();
  rclcpp::spin(tf_checker_node); // Spin the node
  rclcpp::shutdown();
  return 0;
}
