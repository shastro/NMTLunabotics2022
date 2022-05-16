#include <iostream>
#include <ros/ros.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include <sstream>

int main(int argc, char **argv) {

  if ( argc != 3 ){
    std::cout << "Usage: camera_node <topic_name> <port>" << std::endl;
    abort();
  }
  // Initialize node
  ros::init(argc, argv, "camera_pub");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // Create topic
  std::stringstream topic_name;
  topic_name << argv[1];
  image_transport::Publisher pub = it.advertise(topic_name.str(), 1);

  // Open UDP camera stream
  std::stringstream udp_str;
  udp_str << "udp://127.0.0.1:" << argv[2] << "?overrun_nonfatal=1&fifo_size=50000000";
  cv::VideoCapture cap(udp_str.str(), cv::CAP_FFMPEG);

  // Setup loop vars
  cv::Mat frame;

  // Check if camera can be opened
  if (cap.isOpened() == false){
    std::cout << "VideoCapture failed to open" << std::endl;
  }

  // Run publish loop
  while(cap.isOpened()){
    cap >> frame;
    // Create image message
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
  }
  cap.release();


  return 0;
}
