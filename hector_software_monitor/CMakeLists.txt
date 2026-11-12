cmake_minimum_required(VERSION 3.8)
project(hector_software_monitor)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(topic_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_aggregator REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Header-only interface library carrying include dirs and dependencies. Every source file in src/
# defines its own main() and is built as a separate executable below, so they must not be compiled
# into a shared library (duplicate main() fails to link with LTO).
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  diagnostic_aggregator::diagnostic_aggregator
  ${diagnostic_msgs_TARGETS}
  pluginlib::pluginlib
)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Executables
add_executable(active_nodes_checker src/active_nodes_checker.cpp)
# Link against the local library target and external dependencies
target_link_libraries(active_nodes_checker PUBLIC ${PROJECT_NAME})
target_link_libraries(active_nodes_checker PUBLIC rclcpp::rclcpp)

add_executable(active_topics_checker src/active_topics_checker.cpp)
# Link against the local library target and external dependencies
target_link_libraries(active_topics_checker PUBLIC ${PROJECT_NAME})
target_link_libraries(active_topics_checker PUBLIC rclcpp::rclcpp)

add_executable(tf_checker src/tf_checker.cpp)
# Link against the local library target and external dependencies
target_link_libraries(tf_checker PUBLIC ${PROJECT_NAME})
target_link_libraries(tf_checker PUBLIC rclcpp::rclcpp)

add_executable(topic_types_checker src/topic_types_checker.cpp)
# Link against the local library target and external dependencies
target_link_libraries(topic_types_checker PUBLIC ${PROJECT_NAME})
target_link_libraries(topic_types_checker PUBLIC rclcpp::rclcpp)

add_executable(topic_frequency_checker src/topic_frequency_checker.cpp)
# Link against the local library target and external dependencies
target_link_libraries(topic_frequency_checker PUBLIC ${PROJECT_NAME})
target_link_libraries(topic_frequency_checker PUBLIC rclcpp::rclcpp)

# Install Library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Executables
install(TARGETS active_nodes_checker active_topics_checker tf_checker topic_types_checker topic_frequency_checker
  DESTINATION lib/${PROJECT_NAME} # Install executables to package-specific lib folder
)

# Install Headers
install(DIRECTORY include/
  DESTINATION include
)

# Export targets for downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(
  rclcpp
  topic_tools
  tf2_ros
  diagnostic_aggregator
  diagnostic_msgs
  pluginlib
)

# Declare package
ament_package()
