cmake_minimum_required(VERSION 3.5)
project(vesc_ackermann)

# Setting C++ standard to 14
if(NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find sensor_msgs package
find_package(sensor_msgs REQUIRED)

# Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ackermann_to_vesc.cpp
  src/vesc_to_odom.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  ${sensor_msgs_INCLUDE_DIRS}
)

# Register nodes as components
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "vesc_ackermann::AckermannToVesc"
  EXECUTABLE "ackermann_to_vesc_node"
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "vesc_ackermann::VescToOdom"
  EXECUTABLE "vesc_to_odom_node"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
