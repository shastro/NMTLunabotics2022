#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

int main(int argc, char** argv){
  if (argc != 3) {
    std::cout << "Usage: camera_node <addr> <port>" << std::endl;
    abort();
  }

  std::stringstream udp_str;
  udp_str << "udp://"<< argv[1] << ":" << argv[2]
          << "?overrun_nonfatal=1&fifo_size=278876";
  std::cout << udp_str.str() << std::endl;

  cv::VideoCapture cap(udp_str.str(), cv::CAP_FFMPEG);

  // Setup loop vars
  cv::Mat frame;

  // Check if camera can be opened
  if (cap.isOpened() == false) {
    std::cout << "VideoCapture failed to open" << std::endl;
  }

  // Run publish loop
  while (cap.isOpened()) {
    cap >> frame;
    cv::imshow("Camera", frame);
  }
  cap.release();

  return 0;
}
