#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

class WaypointCalculator {
public:
  WaypointCalculator() {
    image_transport::ImageTransport it(_nh);
    it.subscribe("/camera_image", 1, &WaypointCalculator::camera_image_callback,
                 this);
  }

  void camera_image_callback(const sensor_msgs::ImageConstPtr &ros_image) {}

private:
  ros::NodeHandle _nh;
  // TODO: Waypoint position publishers.
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoints");

  WaypointCalculator calc;
  ros::spin();

  ros::shutdown();
}
