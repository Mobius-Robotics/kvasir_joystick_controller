cmake_minimum_required(VERSION 3.8)
project(kvasir_joystick_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
set(MY_DEPENDENCIES
  rclcpp
  kvasir_hw_interface
)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${MY_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

## Executable
add_executable(
  ${PROJECT_NAME} src/${PROJECT_NAME}.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC ${MY_DEPENDENCIES}
)

## Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  PROGRAMS src/kvasir_joystick_reader.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME kvasir_joystick_reader
)

ament_package()
