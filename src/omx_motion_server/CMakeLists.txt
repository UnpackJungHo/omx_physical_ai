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
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# miniconda libcurl/libssl 충돌 방지: 시스템 curl/ssl 명시 링크
set(SYSTEM_LIBRARY_DIRS /usr/lib /usr/local/lib)
if(CMAKE_LIBRARY_ARCHITECTURE)
  list(PREPEND SYSTEM_LIBRARY_DIRS "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
endif()
find_library(SYSTEM_CURL curl HINTS ${SYSTEM_LIBRARY_DIRS} NO_DEFAULT_PATH)
find_library(SYSTEM_SSL ssl HINTS ${SYSTEM_LIBRARY_DIRS} NO_DEFAULT_PATH)
find_library(SYSTEM_CRYPTO crypto HINTS ${SYSTEM_LIBRARY_DIRS} NO_DEFAULT_PATH)

set(SYSTEM_RPATH_DIRS /opt/ros/jazzy/lib /usr/lib)
if(CMAKE_LIBRARY_ARCHITECTURE)
  list(PREPEND SYSTEM_RPATH_DIRS "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
endif()
list(REMOVE_DUPLICATES SYSTEM_RPATH_DIRS)
string(JOIN ";" SYSTEM_RPATH ${SYSTEM_RPATH_DIRS})

add_executable(motion_server src/motion_server.cpp)
# miniconda 라이브러리가 런타임에 끼어들지 못하도록 시스템 경로를 RPATH 에 명시
set_target_properties(motion_server PROPERTIES
  INSTALL_RPATH "${SYSTEM_RPATH}"
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
  control_msgs
  trajectory_msgs
  sensor_msgs
  tf2
  tf2_ros)

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

# ── grasp_detector ─────────────────────────────────────────────────
# Step 1: Dynamixel current 기반 grasp 판단 노드.
add_executable(grasp_detector src/grasp_detector.cpp)
target_compile_features(grasp_detector PUBLIC cxx_std_17)
ament_target_dependencies(grasp_detector
  rclcpp
  omx_interfaces
  sensor_msgs
  std_msgs
  control_msgs)

install(TARGETS motion_server grasp_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# config/grasp_detector.yaml 등 향후 파라미터 파일을 위해 디렉터리 install 준비.
# 현재 디렉터리가 비어있으면 install 이 실패하므로 존재 시에만 설치.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
