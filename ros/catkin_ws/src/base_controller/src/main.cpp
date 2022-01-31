// ROS geometry message dispatcher.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include "main.hpp"

static void quit(int sig);

class Robot {
public:
  Robot(std::vector<NavMotor> motors) { motors_ = std::move(motors); }

  void twist(const geometry_msgs::Twist &msg) {
    const geometry_msgs::Vector3 &linear = msg.linear;
    const geometry_msgs::Vector3 &angular = msg.angular;

    // From https://wiki.ros.org/navigation/Tutorials/RobotSetup,
    // §1.5:
    //
    // > This means there must be a node subscribing to the "cmd_vel"
    // > topic that is capable of taking (vx, vy, vtheta) <==>
    // > (cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)
    // > velocities and converting them into motor commands to send to
    // > a mobile base.
    double forward = linear.x;
    double right = linear.y;
    double rotation = angular.z;

    // We can't do anything with side-to-side motion because our robot
    // has only forward-facing wheels.
    (void)right;

    for (NavMotor &motor : motors_) {
      motor.nav(forward, rotation);
    }
  }

private:
  std::vector<NavMotor> motors_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "base_controller");

  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " <robot-path> <cmd-vel-path>"
              << std::endl;
    std::cout << std::endl;

    std::cout << "robot-path    Path to the robot root." << std::endl;
    std::cout << "cmd-vel-path  Path to a topic generating" << std::endl;
    std::cout << "              `geometry_msgs/Twist` messages." << std::endl;
    quit(1);
  }

  std::string robot_path(argv[1]);
  std::string cmd_vel_path(argv[2]);
  ros::NodeHandle nh;

  // Initialize the navigation motors.
  Robot robot(init_motors(robot_path));

  // Subscribe to the `cmd_vel` topic, whose packet types are
  // `geometry_msgs.msg.Twist`.
  ros::Subscriber sub = nh.subscribe(cmd_vel_path, 1, &Robot::twist, &robot);

  ros::spin();
}

static void quit(int sig) {
  ros::shutdown();
  exit(sig);
}
