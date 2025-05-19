cmake_minimum_required(VERSION 3.8)
project(kvasir_joystick_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
set(MY_DEPENDENCIES
  rclcpp
  kvasir_hw_interface
  geometry_msgs
)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${MY_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

## Joystick executable
add_executable(
  ${PROJECT_NAME} src/${PROJECT_NAME}.cpp
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC ${MY_DEPENDENCIES}
)

## Teleop executable
add_executable(
  kvasir_teleop_controller src/kvasir_teleop_controller.cpp
)
ament_target_dependencies(
  kvasir_teleop_controller PUBLIC ${MY_DEPENDENCIES}
)

## Install
install(TARGETS ${PROJECT_NAME} kvasir_teleop_controller
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
