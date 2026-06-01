cmake_minimum_required(VERSION 3.8)
project(omx_dynamixel_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(omx_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 플러그인 라이브러리 (소스는 Task 2,4,6에서 채운다)
add_library(omx_dynamixel_hw SHARED
  src/control_table.cpp
  src/dxl_bus.cpp
  src/dynamixel_system.cpp
)
target_include_directories(omx_dynamixel_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(omx_dynamixel_hw PUBLIC cxx_std_17)
ament_target_dependencies(omx_dynamixel_hw
  hardware_interface pluginlib rclcpp rclcpp_lifecycle dynamixel_sdk realtime_tools omx_interfaces
  std_srvs)

pluginlib_export_plugin_description_file(hardware_interface omx_dynamixel_hw.xml)

# 실HW ping/scan 스모크 도구
add_executable(dxl_ping_tool src/dxl_ping_tool.cpp)
target_link_libraries(dxl_ping_tool omx_dynamixel_hw)
ament_target_dependencies(dxl_ping_tool dynamixel_sdk)
install(TARGETS dxl_ping_tool DESTINATION lib/${PROJECT_NAME})

# 진단 구독 + 안전정지 watchdog 노드
add_executable(dxl_watchdog_node src/dxl_watchdog_node.cpp)
ament_target_dependencies(dxl_watchdog_node rclcpp omx_interfaces controller_manager_msgs)
target_include_directories(dxl_watchdog_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(TARGETS dxl_watchdog_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS omx_dynamixel_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config OPTIONAL)
install(FILES omx_dynamixel_hw.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(omx_dynamixel_hw)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_table test/test_control_table.cpp src/control_table.cpp)
  target_include_directories(test_control_table PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  ament_add_gtest(test_watchdog_logic test/test_watchdog_logic.cpp)
  target_include_directories(test_watchdog_logic PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_watchdog_logic PRIVATE cxx_std_20)  # designated initializers in test
endif()

ament_package()
