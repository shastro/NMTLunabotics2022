cmake_minimum_required(VERSION 3.0.2)
project(nav_dummy)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs webots_ros)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

add_executable(nav_dummy src/main.cpp)
target_link_libraries(nav_dummy ${catkin_LIBRARIES})

install(TARGETS nav_dummy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
