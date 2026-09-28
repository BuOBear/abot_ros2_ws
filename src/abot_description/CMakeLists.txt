cmake_minimum_required(VERSION 3.8)
project(abot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(
    NAME description_structure
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_description.py"
            "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
