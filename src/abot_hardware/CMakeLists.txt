cmake_minimum_required(VERSION 3.8)
project(abot_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(abot_protocol src/protocol.cpp)
target_include_directories(abot_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(abot_serial src/serial_session.cpp)
target_include_directories(abot_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(abot_serial PUBLIC abot_protocol)

add_executable(base_driver src/base_driver.cpp)
target_link_libraries(base_driver abot_serial)
ament_target_dependencies(base_driver
  rclcpp rclcpp_lifecycle lifecycle_msgs rcl_interfaces geometry_msgs nav_msgs sensor_msgs diagnostic_msgs)

install(TARGETS abot_protocol abot_serial base_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol abot_protocol)
  ament_add_gtest(test_serial_pty test/test_serial_pty.cpp)
  target_link_libraries(test_serial_pty abot_serial util)
  ament_add_pytest_test(test_lifecycle_pty test/test_lifecycle_pty.py
    TIMEOUT 90
    ENV "ABOT_BASE_DRIVER=$<TARGET_FILE:base_driver>")
endif()

ament_package()
