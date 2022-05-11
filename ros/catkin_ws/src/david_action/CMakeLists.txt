cmake_minimum_required(VERSION 3.0.2)
project(david_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
)

add_action_files(
  DIRECTORY action FILES
  Navigation.action
  Pitch.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation src/navigation.cpp)
add_executable(pitch src/pitch.cpp)

target_link_libraries(
  navigation
  ${catkin_LIBRARIES}
)

add_dependencies(
  navigation
  ${david_action_EXPORTED_TARGETS}
)

target_link_libraries(
  pitch
  ${catkin_LIBRARIES}
)

add_dependencies(
  pitch
  ${david_action_EXPORTED_TARGETS}
)
