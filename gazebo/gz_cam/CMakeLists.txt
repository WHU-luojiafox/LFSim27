cmake_minimum_required(VERSION 3.22)
project(gz_cam)

# Apply a local policy floor to legacy third-party CMake files (JsonCpp/GTest).
# The project's own minimum remains 3.22; no command-line workaround is needed.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM OR
      CMAKE_POLICY_VERSION_MINIMUM VERSION_LESS "3.10")
    set(CMAKE_POLICY_VERSION_MINIMUM "3.10")
  endif()
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# Gazebo Fortress
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

message(STATUS "Compiling against Ignition Fortress")


# Include
include_directories(include)

# Create gz_cam lib
add_library(${PROJECT_NAME} SHARED
  src/gz_cam_node.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  robot_interfaces
  image_transport
  ros_gz_bridge
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
)

# Create executable nodes
add_executable(gz_cam_node src/gz_cam_main.cpp)
set_target_properties(gz_cam_node PROPERTIES OUTPUT_NAME gz_cam)
target_link_libraries(gz_cam_node ${PROJECT_NAME})

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} "gz_cam::GzCamNode")

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)
# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executable nodes
install(TARGETS gz_cam_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch res directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
