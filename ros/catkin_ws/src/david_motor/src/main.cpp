// ROS geometry message dispatcher.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include <david_motor/AugerCmd.h>
#include <david_motor/DepthCmd.h>
#include <david_motor/DumperCmd.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "main.hpp"
#include "ros/node_handle.h"

using david_motor::AugerCmd;
using david_motor::DepthCmd;
using david_motor::DumperCmd;
using geometry_msgs::Vector3;
using ros::NodeHandle;
using ros::Subscriber;

// Encoder per degree 0.03
// radians per
// enc per second * radians per enc 1909.859
// Want to take RPM and turn to meters/second
// Have meters per encoder or encoder per meters
// RPM to radians per second times encoder/radians times meter per encoder

using namespace std;

static void quit(int sig);

class Robot {
private:
  ros::Publisher _telemetry;
  ros::NodeHandle _nh;

  Subscriber _vel_sub;
  Subscriber _aug_sub;
  Subscriber _dep_sub;
  Subscriber _dmp_sub;

public:
  Robot(vector<NavMotor> motors, NavMotor *augerMotor, NavMotor *depthLMotor,
        NavMotor *depthRMotor, NavMotor *dumperLMotor, NavMotor *dumperRMotor,
        string cmd_vel_path)
      : _nh() {
    motors_ = move(motors);
    augerMotor_ = augerMotor;
    depthLMotor_ = depthLMotor;
    depthRMotor_ = depthRMotor;
    dumperLMotor_ = dumperLMotor;
    dumperRMotor_ = dumperRMotor;

    _vel_sub = _nh.subscribe(cmd_vel_path, 1, &Robot::twist, this);
    _aug_sub = _nh.subscribe("/cmd_auger", 1, &Robot::auger, this);
    _dep_sub = _nh.subscribe("/cmd_depth", 1, &Robot::depth, this);
    _dmp_sub = _nh.subscribe("/cmd_dumper", 1, &Robot::dumper, this);

    _telemetry =
        _nh.advertise<sensor_msgs::JointState>("/joints/motor_joints", 1000);
  }

  void twist(const geometry_msgs::Twist &msg) {
    const Vector3 &linear = msg.linear;
    const Vector3 &angular = msg.angular;

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

  void auger(const AugerCmd &cmd) {
    double spin = cmd.spin;
    if (augerMotor_) {
      augerMotor_->nav(spin, 0);
    }
  }

  void depth(const DepthCmd &cmd) {
    double vel = cmd.depth_vel;
    if (depthLMotor_) {
      depthLMotor_->nav(-vel, 0);
    }

    if (depthRMotor_) {
      depthRMotor_->nav(-vel, 0);
    }
  }

  void dumper(const DumperCmd &cmd) {
    double vel = cmd.vel;
    if (dumperLMotor_) {
      dumperLMotor_->nav(vel, 0);
    }

    if (dumperRMotor_) {
      dumperRMotor_->nav(vel, 0);
    }
  }

private:
  std::vector<NavMotor> motors_;

  // These are optional motors, each of which needs to be at position
  // 0. They're NavMotors and not just normal motors because I'm
  // rushing.
  NavMotor *augerMotor_;
  NavMotor *depthLMotor_;
  NavMotor *depthRMotor_;
  NavMotor *dumperLMotor_;
  NavMotor *dumperRMotor_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "david_motor");

  if (argc != 3) {
    cout << "Usage: " << argv[0] << " <robot-path> <cmd-vel-path>" << endl
         << endl
         << "robot-path    Path to the robot root." << endl
         << "cmd-vel-path  Path to a topic generating" << endl
         << "              `geometry_msgs/Twist` messages." << endl;

    return EXIT_FAILURE;
  }

  string robot_path = argv[1];
  string cmd_vel_path = argv[2];

  // Initialize the navigation motors.
  NavMotor *augerMotor = nullptr;
  NavMotor *depthLMotor = nullptr;
  NavMotor *depthRMotor = nullptr;
  NavMotor *dumperLMotor = nullptr;
  NavMotor *dumperRMotor = nullptr;
  vector<NavMotor> motors =
      init_motors(robot_path, augerMotor, depthLMotor, depthRMotor,
                  dumperLMotor, dumperRMotor);
  Robot robot(move(motors), augerMotor, depthLMotor, depthRMotor, dumperLMotor,
              dumperRMotor, cmd_vel_path);

  // Subscribe to the topics. Kinda scuffed because I wrote cmd_vel
  // before anything else so it uses a worse convention, but I don't
  // have time to fix it because competiton is in 2 days.

  ros::spin();
}

static void quit(int sig) {
  ros::shutdown();
  exit(sig);
}
