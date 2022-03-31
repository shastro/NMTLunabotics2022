cmake_minimum_required(VERSION 2.8)
project(aruco_testproject)
find_package(OpenCV REQUIRED )
add_executable(testboard testboard.cpp)
target_link_libraries(testboard ${OpenCV_LIBS} )
add_executable(detect_board detect_board.cpp)
target_link_libraries(detect_board ${OpenCV_LIBS})



# cmake .. -Daruco_DIR=/home/shastro/installdump/aruco-3.1.12/build -DOpenCV_DIR=/home/shastro/installdump/opencv/build
