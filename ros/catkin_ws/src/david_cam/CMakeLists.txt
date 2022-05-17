cmake_minimum_required(VERSION 3.0.2)
project(david_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  message_generation
  cv_bridge
)

find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs image_transport cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_node src/camera_node.cpp)
add_executable(camera_img_service src/camera_service.cpp)

target_link_libraries(
  camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  camera_img_service
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_dependencies(
  camera_node
  ${david_cam_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  camera_img_service
  ${david_cam_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)


