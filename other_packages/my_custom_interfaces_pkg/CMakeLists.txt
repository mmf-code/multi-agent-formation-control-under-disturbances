cmake_minimum_required(VERSION 3.8)
project(my_custom_interfaces_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FormationState.msg"
  "msg/MetricsData.msg"
  "msg/ControllerParams.msg"
  "msg/NeighborPosition.msg"
  "msg/NeighborInfo.msg"
  "srv/UpdateFormation.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
