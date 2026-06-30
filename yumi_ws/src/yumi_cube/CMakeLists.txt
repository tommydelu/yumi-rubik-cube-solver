cmake_minimum_required(VERSION 3.10)
project(yumi_cube)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(yumi_cube_node src/main.cpp src/primitives.cpp)
ament_target_dependencies(yumi_cube_node rclcpp geometry_msgs std_msgs tf2 tf2_geometry_msgs)
target_include_directories(yumi_cube_node PRIVATE include)

add_executable(sequence_node src/sequence.cpp)
ament_target_dependencies(sequence_node rclcpp std_msgs)

install(TARGETS
  yumi_cube_node
  sequence_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  config
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
