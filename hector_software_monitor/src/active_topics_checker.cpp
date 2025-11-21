#include <algorithm>
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

class ActiveTopicsChecker : public rclcpp::Node
{
public:
  ActiveTopicsChecker() : Node( "active_topics_checker" )
  {
    diagnostics_publisher_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>( "/diagnostics", 10 );

    // Declare and get parameters
    this->declare_parameter<std::vector<std::string>>( "active_topics", std::vector<std::string>() );
    std::vector<std::string> topics_to_be_checked;

    // Add a small delay to allow parameters to be set externally, if necessary.
    // In a real scenario, consider using parameter services or waiting for parameter events.
    RCLCPP_INFO( this->get_logger(), "Waiting briefly for parameters..." );
    rclcpp::Rate param_wait_rate( 2.0 );            // Check twice a second
    for ( int i = 0; i < 5 && rclcpp::ok(); ++i ) { // Wait up to 2.5 seconds
      if ( this->get_parameter( "active_topics", topics_to_be_checked ) ) {
        if ( !topics_to_be_checked.empty() ) {
          break;
        }
      }
      param_wait_rate.sleep();
    }

    if ( topics_to_be_checked.empty() ) {
      RCLCPP_ERROR( this->get_logger(),
                    "[ActiveTopicsChecker] Could not get non-empty \"active_topics\" parameter." );
      rclcpp::shutdown();
      return;
    }

    topics_to_be_checked_ = topics_to_be_checked;

    // Print loaded params
    std::stringstream ss;
    for ( const auto &topic_name : topics_to_be_checked_ ) ss << std::endl << "> " << topic_name;
    RCLCPP_INFO( this->get_logger(), "[ActiveTopicsChecker] Checking following topics:%s",
                 ss.str().c_str() );

    // Create timer that triggers at 1Hz
    timer_ = this->create_wall_timer( std::chrono::seconds( 1 ),
                                      std::bind( &ActiveTopicsChecker::check_topics, this ) );
  }

private:
  void check_topics()
  {
    auto topics_and_types = this->get_topic_names_and_types();

    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    diagnostic_array.header.stamp = this->get_clock()->now();

    for ( const auto &topic_to_be_checked : topics_to_be_checked_ ) {
      std::vector<rclcpp::TopicEndpointInfo> publishers =
          this->get_publishers_info_by_topic( topic_to_be_checked );
      std::vector<rclcpp::TopicEndpointInfo> subscriptions;

      diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
      diagnostic_status.name = "active_topics" + topic_to_be_checked; // e.g., active_topics/my_topic

      diagnostic_status.level = publishers.empty() ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                                   : diagnostic_msgs::msg::DiagnosticStatus::OK;
      diagnostic_status.message = publishers.empty()
                                      ? "No publishers"
                                      : "Pubs: " + std::to_string( publishers.size() ) +
                                            ", Subs: " + std::to_string( subscriptions.size() );

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "topic";
      kv.value = topic_to_be_checked;
      diagnostic_status.values.push_back( kv );

      kv.key = "publishers_count";
      kv.value = std::to_string( publishers.size() );
      diagnostic_status.values.push_back( kv );

      kv.key = "subscriptions_count";
      kv.value = std::to_string( subscriptions.size() );
      diagnostic_status.values.push_back( kv );

      kv.key = "publishers_nodes";
      kv.value.clear();
      for ( const auto &pub_info : publishers ) { kv.value += pub_info.node_name() + " "; }
      diagnostic_status.values.push_back( kv );

      kv.key = "subscriptions_nodes";
      kv.value.clear();
      for ( const auto &sub_info : subscriptions ) { kv.value += sub_info.node_name() + " "; }
      diagnostic_status.values.push_back( kv );

      diagnostic_array.status.push_back( diagnostic_status );
    }

    diagnostics_publisher_->publish( diagnostic_array );
  }

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> topics_to_be_checked_;
};

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<ActiveTopicsChecker>();
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}
