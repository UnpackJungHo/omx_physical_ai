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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToNamed.action"
  "action/MoveToPose.action"
  "action/GripperCommand.action"
  "action/PickPlace.action"
  "msg/BlockPose.msg"
  "srv/GetBlockPoses.srv"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
