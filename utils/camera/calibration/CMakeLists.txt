cmake_minimum_required(VERSION 3.0.2)
project(camera_utils)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  arucolib/src
)

add_executable(take_picture take_pic.cpp)

target_link_libraries(
  calibrate_cam
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  take_picture
  ${OpenCV_LIBRARIES}
)
