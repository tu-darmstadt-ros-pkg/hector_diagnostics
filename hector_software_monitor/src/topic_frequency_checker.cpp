#include <chrono>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <iomanip>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

/**
 * Publishes the receive rate of each configured topic on /diagnostics and compares it to [min_hz, max_hz].
 *
 * It uses generic subscriptions (serialized messages are never deserialized), but every subscription
 * still receives a copy of the data.
 */
class TopicFrequencyChecker : public rclcpp::Node
{
public:
  TopicFrequencyChecker() : Node( "topic_frequency_checker" )
  {
    diagnostics_publisher_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>( "/diagnostics", 10 );

    // Parallel lists, one entry per topic (ROS 2 parameters cannot hold a list of dicts).
    this->declare_parameter<std::vector<std::string>>( "frequency_topics", std::vector<std::string>() );
    this->declare_parameter<std::vector<double>>( "frequency_min_hz", std::vector<double>() );
    this->declare_parameter<std::vector<double>>( "frequency_max_hz", std::vector<double>() );
    this->declare_parameter<std::vector<double>>( "frequency_timeouts", std::vector<double>() );
    // Length of the sliding window the rate is averaged over
    window_ = this->declare_parameter<double>( "frequency_window", 5.0 );

    std::vector<std::string> topics;
    std::vector<double> min_hz, max_hz, timeouts;

    // Add a small delay to allow parameters to be set externally, if necessary.
    RCLCPP_INFO( this->get_logger(), "Waiting briefly for parameters..." );
    rclcpp::Rate param_wait_rate( 2.0 );            // Check twice a second
    for ( int i = 0; i < 5 && rclcpp::ok(); ++i ) { // Wait up to 2.5 seconds
      topics = this->get_parameter( "frequency_topics" ).as_string_array();
      if ( !topics.empty() )
        break;
      param_wait_rate.sleep();
    }
    min_hz = this->get_parameter( "frequency_min_hz" ).as_double_array();
    max_hz = this->get_parameter( "frequency_max_hz" ).as_double_array();
    timeouts = this->get_parameter( "frequency_timeouts" ).as_double_array();

    if ( topics.empty() ) {
      RCLCPP_ERROR( this->get_logger(),
                    "[TopicFrequencyChecker] Could not get non-empty \"frequency_topics\" parameter." );
      rclcpp::shutdown();
      return;
    }
    if ( min_hz.size() != topics.size() || max_hz.size() != topics.size() ||
         ( !timeouts.empty() && timeouts.size() != topics.size() ) ) {
      RCLCPP_ERROR( this->get_logger(),
                    "[TopicFrequencyChecker] \"frequency_topics\" (%zu), \"frequency_min_hz\" (%zu), "
                    "\"frequency_max_hz\" (%zu) and \"frequency_timeouts\" (%zu, optional) must have the "
                    "same length.",
                    topics.size(), min_hz.size(), max_hz.size(), timeouts.size() );
      rclcpp::shutdown();
      return;
    }

    std::stringstream ss;
    for ( size_t i = 0; i < topics.size(); ++i ) {
      std::string name = topics[i][0] == '/' ? topics[i] : '/' + topics[i];
      if ( min_hz[i] > max_hz[i] || min_hz[i] < 0 )
        RCLCPP_WARN( this->get_logger(), "[TopicFrequencyChecker] %s: invalid range %.2f-%.2f Hz",
                     name.c_str(), min_hz[i], max_hz[i] );
      TopicData data;
      data.min_hz = min_hz[i];
      data.max_hz = max_hz[i];
      data.timeout = timeouts.empty() ? 3.0 : timeouts[i];
      topics_[name] = data;
      ss << std::endl
         << "> " << name << " | min: " << data.min_hz << ", max: " << data.max_hz
         << ", timeout: " << data.timeout;
    }
    RCLCPP_INFO( this->get_logger(), "[TopicFrequencyChecker] Watching following topic-frequency pairs:%s",
                 ss.str().c_str() );

    // Create timer that triggers at 1Hz
    timer_ = this->create_wall_timer( std::chrono::seconds( 1 ),
                                      std::bind( &TopicFrequencyChecker::check_topics, this ) );
  }

private:
  struct TopicData {
    double min_hz = 0.0;
    double max_hz = 0.0;
    double timeout = 3.0;
    std::string type;
    rclcpp::GenericSubscription::SharedPtr subscription;
    std::deque<rclcpp::Time> stamps; // receive times within the window
  };

  // Subscribe once the topic (and with it its type) shows up in the graph.
  void subscribe_new_topics()
  {
    auto topics_and_types = this->get_topic_names_and_types();
    for ( auto &[name, data] : topics_ ) {
      if ( data.subscription )
        continue;
      auto it = topics_and_types.find( name );
      if ( it == topics_and_types.end() || it->second.empty() )
        continue;
      data.type = it->second.front();
      // Best effort matches reliable and best-effort publishers alike
      data.subscription = this->create_generic_subscription(
          name, data.type, rclcpp::SensorDataQoS(),
          [this, name]( std::shared_ptr<rclcpp::SerializedMessage> ) {
            topics_[name].stamps.push_back( this->now() );
          } );
      RCLCPP_INFO( this->get_logger(), "[TopicFrequencyChecker] Subscribed to %s [%s]", name.c_str(),
                   data.type.c_str() );
    }
  }

  void check_topics()
  {
    subscribe_new_topics();

    const rclcpp::Time now = this->now();
    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
    diagnostic_array.header.stamp = now;

    for ( auto &[name, data] : topics_ ) {
      while ( !data.stamps.empty() && ( now - data.stamps.front() ).seconds() > window_ )
        data.stamps.pop_front();

      diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
      diagnostic_status.name = "topic_frequency::" + name;

      double frequency = 0.0;
      if ( data.stamps.size() >= 2 )
        frequency = ( data.stamps.size() - 1 ) / ( data.stamps.back() - data.stamps.front() ).seconds();

      std::stringstream ss;
      if ( !data.subscription ) {
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diagnostic_status.message = "Topic not advertised";
      } else if ( data.stamps.empty() || ( now - data.stamps.back() ).seconds() > data.timeout ) {
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        ss << "No message for " << data.timeout << " s";
        diagnostic_status.message = ss.str();
      } else if ( data.stamps.size() < 2 ) {
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diagnostic_status.message = "Not enough messages yet";
      } else if ( frequency < data.min_hz || frequency > data.max_hz ) {
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        ss << std::fixed << std::setprecision( 2 ) << "Is: " << frequency << " Hz, should be: " << data.min_hz
           << "-" << data.max_hz << " Hz";
        diagnostic_status.message = ss.str();
      } else {
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        ss << std::fixed << std::setprecision( 2 ) << frequency << " Hz";
        diagnostic_status.message = ss.str();
      }

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "topic";
      kv.value = name;
      diagnostic_status.values.push_back( kv );

      kv.key = "frequency";
      std::stringstream freq;
      freq << std::fixed << std::setprecision( 2 ) << frequency << " Hz";
      kv.value = freq.str();
      diagnostic_status.values.push_back( kv );

      kv.key = "expected";
      std::stringstream expected;
      expected << std::fixed << std::setprecision( 2 ) << data.min_hz << "-" << data.max_hz << " Hz";
      kv.value = expected.str();
      diagnostic_status.values.push_back( kv );

      kv.key = "publishers_count";
      kv.value = std::to_string( this->count_publishers( name ) );
      diagnostic_status.values.push_back( kv );

      diagnostic_array.status.push_back( diagnostic_status );
    }

    diagnostics_publisher_->publish( diagnostic_array );
  }

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, TopicData> topics_;
  double window_;
};

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  auto node = std::make_shared<TopicFrequencyChecker>();
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}
