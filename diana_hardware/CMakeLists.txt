cmake_minimum_required(VERSION 3.8)
project(diana_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(diana_hardware SHARED
        src/robot.cpp
        src/diana_hardware_interface.cpp
)

target_include_directories(
  diana_hardware
  PRIVATE
    include
)

ament_target_dependencies(
        diana_hardware
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_action
)

# pluginlib_export_plugin_description_file(hardware_interface diana_hardware.xml)

install(
  TARGETS diana_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(
  include
)

ament_export_libraries(
  diana_hardware
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()
