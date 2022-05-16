#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

class WaypointCalculator {
public:
  WaypointCalculator(char *topic_name) {
    image_transport::ImageTransport it(_nh);
    it.subscribe(topic_name, 1, &WaypointCalculator::camera_image_callback,
                 this);
  }

  void camera_image_callback(const sensor_msgs::ImageConstPtr &ros_image) {

  }


private:
  ros::NodeHandle _nh;
  // TODO: Waypoint position publishers.
};

int main(int argc, char **argv) {

  if (argc != 2) {
    cout << "Usage waypoints <image_topic>" << endl;
    abort();
  }
  ros::init(argc, argv, "waypoints");

  WaypointCalculator calc(argv[1]);
  ros::spin();

  ros::shutdown();
}
