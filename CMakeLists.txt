# Build definition for the Simple RViz viewer package.
cmake_minimum_required(VERSION 3.8)
project(rp_simple_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc)

# Viewer executable: subscribes to existing ROS topics and draws them.
add_executable(simple_rviz_node
  src/simple_rviz_node.cpp
  src/simple_rviz_map.cpp
  src/simple_rviz_rendering.cpp
  src/simple_rviz_interaction.cpp
)
ament_target_dependencies(simple_rviz_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_include_directories(simple_rviz_node PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(simple_rviz_node ${OpenCV_LIBS})

add_executable(map_server_node
  src/map_server_node.cpp
)
ament_target_dependencies(map_server_node
  rclcpp
  nav_msgs
)
target_include_directories(map_server_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(map_server_node ${OpenCV_LIBS})

add_executable(demo_robot_node
  src/demo_robot_node.cpp
)
ament_target_dependencies(demo_robot_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_include_directories(demo_robot_node PRIVATE
  include
)

install(
  TARGETS simple_rviz_node map_server_node demo_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and parameters with the package.
install(
  DIRECTORY assets config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
