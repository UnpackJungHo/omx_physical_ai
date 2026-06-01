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

# 플러그인 라이브러리 (소스는 Task 2,4,6에서 채운다)
add_library(omx_dynamixel_hw SHARED
  src/control_table.cpp
)
target_include_directories(omx_dynamixel_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(omx_dynamixel_hw PUBLIC cxx_std_17)
ament_target_dependencies(omx_dynamixel_hw
  hardware_interface pluginlib rclcpp rclcpp_lifecycle dynamixel_sdk realtime_tools omx_interfaces)

# pluginlib export 는 Task 6 에서 클래스가 생긴 뒤 활성화한다.
# pluginlib_export_plugin_description_file(hardware_interface omx_dynamixel_hw.xml)

install(TARGETS omx_dynamixel_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(omx_dynamixel_hw)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_table test/test_control_table.cpp src/control_table.cpp)
  target_include_directories(test_control_table PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
