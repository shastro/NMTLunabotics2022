cmake_minimum_required(VERSION 3.0.2)
project(david_waypoints)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS sensor_msgs image_transport cv_bridge tf2 tf2_ros
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  arucolib/src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(waypoints src/waypoints.cpp)

target_link_libraries(
  waypoints
  aruco
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(
  waypoints
  ${david_cam_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
