cmake_minimum_required(VERSION 3.0.2)
project(pitch_controller)

find_package(
  catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  pitch_controller
  src/main.cpp
)

target_link_libraries(
  pitch_controller
  ${catkin_LIBRARIES}
)

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

install(
  TARGETS pitch_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
