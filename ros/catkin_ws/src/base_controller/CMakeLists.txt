cmake_minimum_required(VERSION 3.0.2)
project(base_controller)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs webots_ros)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

add_executable(base_controller src/main.cpp src/webots.cpp)
target_link_libraries(base_controller ${catkin_LIBRARIES})

install(TARGETS base_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
