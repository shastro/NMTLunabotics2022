
#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"
#include "ros/service_server.h"
#include "sensor_msgs/Image.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include<image_transport/image_transport.h>
#include<opencv2/highgui/highgui.hpp>
#include<cv_bridge/cv_bridge.h>
#include <sstream>
#include <thread>

using namespace std;

sensor_msgs::ImageConstPtr global_msg;
image_transport::Publisher pub;

bool service_callback(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res){
  pub.publish(global_msg);
  return true;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg){
  // Write image to new topic
  global_msg = msg;
}

int main(int argc, char **argv) {

  // Initialize node
  ros::init(argc, argv, "camera_service");

  // Parse private params
  if (argc != 2) {
    std::cout << "Usage: camera_service <camera_topic_name>" << std::endl;
    abort();
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // Subscribe to topic
  std::stringstream cam_topic_name;
  cam_topic_name << argv[1];
  image_transport::Subscriber sub = it.subscribe(cam_topic_name.str(), 1, image_callback);

  // Initialize publisher
  std::stringstream pub_topic_name;
  pub_topic_name << cam_topic_name.str() << "_recent";
  pub = it.advertise(pub_topic_name.str(), 1);

  // Create service server
  std::stringstream service_name;
  service_name << cam_topic_name.str() << "_srv";
  ros::ServiceServer service = nh.advertiseService(service_name.str(), service_callback);

  ROS_INFO("Ready for image service calls");

  ros::spin();

  return 0;
}
