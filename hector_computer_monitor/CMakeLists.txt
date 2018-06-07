cmake_minimum_required(VERSION 2.8.3)
project(hector_computer_monitor)


find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES argo_computer_monitor
#  CATKIN_DEPENDS diagnostic_msgs rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")

catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
