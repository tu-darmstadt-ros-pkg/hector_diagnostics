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

# Define sources and headers
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/${PROJECT_NAME}/*.h")

# Library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  topic_tools
  tf2_ros
  diagnostic_aggregator
  diagnostic_msgs
  pluginlib
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Executables
add_executable(active_nodes_checker src/active_nodes_checker.cpp)
# Link against the local library target and external dependencies
target_link_libraries(active_nodes_checker PUBLIC ${PROJECT_NAME})
ament_target_dependencies(active_nodes_checker PUBLIC rclcpp)

add_executable(tf_checker src/tf_checker.cpp)
# Link against the local library target and external dependencies
target_link_libraries(tf_checker PUBLIC ${PROJECT_NAME})
ament_target_dependencies(tf_checker PUBLIC rclcpp)

# Install Library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Executables
install(TARGETS active_nodes_checker tf_checker
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
