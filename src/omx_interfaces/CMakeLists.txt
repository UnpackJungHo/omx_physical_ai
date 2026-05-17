cmake_minimum_required(VERSION 3.8)
project(omx_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToNamed.action"
  "action/MoveToPose.action"
  "action/MoveToJoints.action"
  "action/GripperCommand.action"
  "action/PickPlace.action"
  "msg/BlockPose.msg"
  "msg/KeypointDetection.msg"
  "srv/GetBlockPoses.srv"
  "srv/GetKeypointDetections.srv"
  "srv/CheckGrasp.srv"
  DEPENDENCIES std_msgs geometry_msgs action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
