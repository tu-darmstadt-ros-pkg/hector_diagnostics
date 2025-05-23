# Hector Diagnostics
This is a collection of [diagnostics](https://github.com/ros/diagnostics/tree/ros2)-related packages.

## hector_software_monitor
### ActiveNodesChecker
Takes as argument a list of nodes and reports for each node if it exists or not.

`ros2 run hector_software_monitor active_nodes_checker --ros-args -p active_nodes:=[node1,node2,...]`

### TopicTypesChecker
Checks if for any topic more than one type is registered.

`ros2 run hector_software_monitor topic_types_checker`

### TF_Checker
Checks if given tf transformations can be retreived within the specified timeout.

Params:
- tf_source_frames
- tf_target_frames
- tf_timeouts

`ros2 run hector_software_monitor tf_checker --ros-args -p tf_source_frames:=[odom] -p tf_target_frames:=[base_link] -p tf_timeouts:=[1.0]`