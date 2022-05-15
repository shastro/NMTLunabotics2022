cmake_minimum_required(VERSION 3.0.2)
project(david_hello)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(david_hello src/main.cpp)

target_link_libraries(
  david_hello
  ${catkin_LIBRARIES}
)
