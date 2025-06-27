cmake_minimum_required(VERSION 3.8)
project(hector_diagnostic_aggregator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_aggregator REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# This is the analyzer plugin, built as a shared library
add_library(${PROJECT_NAME}_analyzers SHARED
  src/hardware_analyzer.cpp
)
target_include_directories(${PROJECT_NAME}_analyzers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_analyzers PUBLIC
  diagnostic_aggregator
  pluginlib
  rclcpp
)
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}_analyzers PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the plugin library
install(
  TARGETS ${PROJECT_NAME}_analyzers
  EXPORT export_${PROJECT_NAME}_analyzers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the plugin description file so the aggregator can find it
pluginlib_export_plugin_description_file(diagnostic_aggregator plugin_description.xml)

# Export for downstream packages
ament_export_targets(export_${PROJECT_NAME}_analyzers HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  diagnostic_aggregator
  pluginlib
  rclcpp
)

ament_package()