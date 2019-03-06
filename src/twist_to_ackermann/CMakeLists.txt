cmake_minimum_required(VERSION 2.8.3)
project(twist_to_ackermann)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_to_ackermann_node src/twist_to_ackermann.cpp)

add_dependencies(twist_to_ackermann_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(twist_to_ackermann_node ${catkin_LIBRARIES})
