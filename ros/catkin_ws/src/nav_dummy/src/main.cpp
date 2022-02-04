// ROS geometry message dispatcher.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <assert.h>
#include <math.h>
#include <unistd.h>

#include "main.hpp"

static void quit(int sig);

int main(int argc, char **argv) {
  ros::init(argc, argv, "nav_dummy");

  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <publishing-path>" << std::endl;
    quit(1);
  }

  std::string path = argv[1];
  ros::NodeHandle nh;

  // Generate the publisher.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(path, 5);
  std::cout << "DEBUG: Advertising " << path << std::endl;
  assert(!!pub);

  double theta = 0;
  while (true) {
    // Delay in seconds between messages.
    const double delay = 0.1;

    // Annoyingly enough, this doesn't allow C-c quitting for some
    // reason unless we explicitly allow it. Thanks, Linux.
    if (usleep((useconds_t)(1000000 * delay)) != 0) {
      std::cout << "Terminating due to signal";
      quit(1);
    }

    geometry_msgs::Twist twist;

    // Use sinusoids to generate random-ish but smooth movement.
    twist.linear.x = sin(theta * 1.0 + 0.0);
    twist.linear.y = sin(theta * 1.1 + 0.5);
    twist.linear.z = sin(theta * 1.2 + 1.0);
    twist.angular.x = sin(theta * 1.3 + 1.5);
    twist.angular.y = sin(theta * 1.4 + 2.0);
    twist.angular.z = sin(theta * 1.5 + 2.5);

    theta += 0.01;

    pub.publish(twist);
  }
}

static void quit(int sig) {
  ros::shutdown();
  exit(sig);
}
