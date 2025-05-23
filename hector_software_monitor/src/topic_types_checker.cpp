#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

class TopicTypesChecker : public rclcpp::Node
{
public:
  TopicTypesChecker() : Node( "topic_types_checker" )
  {
    diagnostics_publisher_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>( "/diagnostics", 10 );

    timer_ = this->create_wall_timer( std::chrono::seconds( 1 ),
                                      std::bind( &TopicTypesChecker::check_topic_types, this ) );
  }

private:
  void check_topic_types()
  {
    auto topic_names_and_types = this->get_topic_names_and_types();
    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "TopicTypeCheck";
    status.hardware_id = "topic_types_checker";
    diagnostic_array.header.stamp = this->get_clock()->now();

    for ( const auto &pair : topic_names_and_types ) {
      if ( pair.second.size() > 1 ) {
        std::ostringstream oss;
        for ( const auto &type : pair.second ) { oss << type << " "; }
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = pair.first;
        kv.value = oss.str();
        status.values.push_back( kv );
      }
    }

    status.level = ( status.values.empty() ) ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                             : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message =
        ( status.values.empty() )
            ? "All topics have a single type"
            : "Multiple types detected for " + std::to_string( status.values.size() ) + " topics";
    diagnostic_array.status.push_back( status );
    diagnostics_publisher_->publish( diagnostic_array );
  }

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<TopicTypesChecker>();
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}