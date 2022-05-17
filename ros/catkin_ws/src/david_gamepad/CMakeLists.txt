cmake_minimum_required(VERSION 3.0.2)
project(david_gamepad)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  david_motor
)

catkin_package(
  CATKIN_DEPENDS
  david_motor
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(david_gamepad src/main.cpp src/joystick/joystick.cpp)

target_link_libraries(
  david_gamepad
  ${catkin_LIBRARIES}
)
