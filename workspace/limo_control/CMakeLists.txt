cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Create executables
add_executable(p_controller src/p_controller.cpp)
ament_target_dependencies(p_controller rclcpp geometry_msgs nav_msgs)

add_executable(performance_logger src/performance_logger.cpp)
ament_target_dependencies(performance_logger rclcpp geometry_msgs nav_msgs)

# Install executables
install(TARGETS p_controller performance_logger
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()