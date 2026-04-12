cmake_minimum_required(VERSION 3.8)
project(omx_motion_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(omx_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# miniconda libcurl/libssl 충돌 방지: 시스템 curl/ssl 명시 링크
find_library(SYSTEM_CURL curl HINTS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
find_library(SYSTEM_SSL ssl HINTS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
find_library(SYSTEM_CRYPTO crypto HINTS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)

add_executable(motion_server src/motion_server.cpp)
# miniconda 라이브러리가 런타임에 끼어들지 못하도록 시스템 경로를 RPATH 에 명시
set_target_properties(motion_server PROPERTIES
  INSTALL_RPATH "/usr/lib/x86_64-linux-gnu;/opt/ros/jazzy/lib"
  BUILD_WITH_INSTALL_RPATH TRUE)
target_include_directories(motion_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_server PUBLIC cxx_std_17)
ament_target_dependencies(motion_server
  rclcpp
  rclcpp_action
  omx_interfaces
  moveit_ros_planning_interface
  geometry_msgs
  control_msgs)

# 시스템 curl/ssl 명시 링크 (miniconda 라이브러리 오염 방지)
if(SYSTEM_CURL)
  target_link_libraries(motion_server ${SYSTEM_CURL})
endif()
if(SYSTEM_SSL)
  target_link_libraries(motion_server ${SYSTEM_SSL})
endif()
if(SYSTEM_CRYPTO)
  target_link_libraries(motion_server ${SYSTEM_CRYPTO})
endif()

install(TARGETS motion_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
