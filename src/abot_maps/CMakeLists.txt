cmake_minimum_required(VERSION 3.8)
project(abot_maps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)
install(DIRECTORY layouts/
  DESTINATION share/${PROJECT_NAME}/layouts
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(
    NAME installed_map_pairs
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_installed_maps.py"
            "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}"
  )
endif()

ament_package()
