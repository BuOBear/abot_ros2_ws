cmake_minimum_required(VERSION 3.8)
project(abot_accessory)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(accessory_serial src/serial_device.cpp)
target_include_directories(accessory_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(accessory_driver src/accessory_driver.cpp)
target_link_libraries(accessory_driver accessory_serial)
ament_target_dependencies(accessory_driver rclcpp std_srvs)

install(TARGETS accessory_serial accessory_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)

  ament_add_gtest(test_serial_pty test/test_serial_pty.cpp)
  target_link_libraries(test_serial_pty accessory_serial util)

  ament_add_pytest_test(test_services_pty test/test_services_pty.py
    TIMEOUT 90
    ENV "ABOT_ACCESSORY_DRIVER=$<TARGET_FILE:accessory_driver>")
endif()

ament_package()
