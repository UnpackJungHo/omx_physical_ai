cmake_minimum_required(VERSION 3.8)
project(omx_perception_world_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(omx_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(box_cup_world_pose_node
  src/box_cup_world_pose_node.cpp
)

target_include_directories(box_cup_world_pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(box_cup_world_pose_node PUBLIC cxx_std_17)

ament_target_dependencies(box_cup_world_pose_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  builtin_interfaces
  omx_interfaces
)

target_link_libraries(box_cup_world_pose_node
  ${OpenCV_LIBS}
  yaml-cpp
)

install(TARGETS box_cup_world_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_helpers
    test/test_helpers.cpp
  )

  target_include_directories(test_helpers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_helpers
    ${OpenCV_LIBS}
  )
endif()

ament_package()
