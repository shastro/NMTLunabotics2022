cmake_minimum_required(VERSION 3.0.2)
project(base_controller)

if($ENV{LOGNAME} STREQUAL pi)
  # We're on the actual robot; link to Teknic.
  find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)
  include_directories(include ${catkin_INCLUDE_DIRS} ../../../../teknic/inc/inc-pub)
  add_executable(base_controller src/main.cpp src/teknic.cpp src/nav_motor.cpp)
  target_link_libraries(base_controller ${catkin_LIBRARIES} -lsFoundation20)
else()
  # We're on the dev machine; link to Webots.
  find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs webots_ros)
  include_directories(include ${catkin_INCLUDE_DIRS})
  add_executable(base_controller src/main.cpp src/webots.cpp src/nav_motor.cpp)
  target_link_libraries(base_controller ${catkin_LIBRARIES})
endif()


link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

install(TARGETS base_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
