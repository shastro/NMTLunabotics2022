cmake_minimum_required(VERSION 3.0.2)
project(david_pitch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  PitchCmd.msg
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

add_executable(pitch src/main.cpp src/gpio_lib/rpi_gpio.cpp)

target_link_libraries(
  pitch
  ${catkin_LIBRARIES}
)

add_dependencies(
  pitch
  ${david_pitch_EXPORTED_TARGETS}
)
