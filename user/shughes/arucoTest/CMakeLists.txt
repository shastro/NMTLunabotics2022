cmake_minimum_required(VERSION 2.8)
project(aruco_testproject)
find_package(aruco REQUIRED )
find_package(OpenCV REQUIRED )
add_executable(aruco_simple aruco_simple.cpp)
target_link_libraries(aruco_simple  aruco)
target_link_libraries(aruco_simple ${OpenCV_LIBS} )

# cmake .. -Daruco_DIR=/home/shastro/installdump/aruco-3.1.12/build -DOpenCV_DIR=/home/shastro/installdump/opencv/build