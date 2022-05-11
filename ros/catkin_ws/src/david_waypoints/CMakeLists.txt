cmake_minimum_required(VERSION 3.0.2)
project(david_waypoints)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoints src/waypoints.cpp)

target_link_libraries(
  navigation
  ${catkin_LIBRARIES}
)

add_dependencies(
  navigation
  ${david_action_EXPORTED_TARGETS}
)
