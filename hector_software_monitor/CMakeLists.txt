cmake_minimum_required(VERSION 3.0.2)
project(hector_software_monitor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp topic_tools tf2_ros diagnostic_aggregator diagnostic_msgs pluginlib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp topic_tools diagnostic_aggregator diagnostic_msgs pluginlib
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/analyzers/simple_analyzer.h
  include/${PROJECT_NAME}/topic_frequency_checker.h
)

set(SOURCES
  src/analyzers/simple_analyzer.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

## Declare a cpp executable
add_executable(active_nodes_checker src/active_nodes_checker.cpp)
add_executable(topic_frequency_checker src/topic_frequency_checker.cpp)
add_executable(tf_checker src/tf_checker.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(active_nodes_checker ${PROJECT_NAME})
target_link_libraries(topic_frequency_checker ${PROJECT_NAME})
target_link_libraries(tf_checker ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} active_nodes_checker tf_checker topic_frequency_checker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  analyzer_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
