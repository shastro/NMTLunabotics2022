// ROS geometry message dispatcher.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <ros/ros.h>
#include <webots_ros/set_float.h>

#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include "main.hpp"

// using namespace std;


// TeleopLunabotics::TeleopLunabotics(string path) : robot_path_(path) {
//   wheels_.push_back(
//       NavMotor(Motor(path + "/motor1"), pair<double, double>(1.0, 0.0)));
//   wheels_.push_back(
//       NavMotor(Motor(path + "/motor2"), pair<double, double>(1.0, 0.0)));
//   wheels_.push_back(
//       NavMotor(Motor(path + "/motor3"), pair<double, double>(0.0, 1.0)));
//   wheels_.push_back(
//       NavMotor(Motor(path + "/motor4"), pair<double, double>(0.0, 1.0)));
//   wheels_.push_back(
//       NavMotor(Motor(path + "/motor5"), pair<double, double>(1.0, 0.0)));
//   wheels_.push_back(
//       NavMotor(Motor(path + "/motor6"), pair<double, double>(1.0, 0.0)));
//   wheels_.push_back(
//       NavMotor(Motor(path + "/motor7"), pair<double, double>(0.0, 1.0)));
//   wheels_.push_back(
//       NavMotor(Motor(path + "/motor8"), pair<double, double>(0.0, 1.0)));
// }

// void quit(int sig) {
//   (void)sig;
//   input.shutdown();
//   ros::shutdown();
//   exit(0);
// }

int main(int argc, char **argv) {
  // ros::init(argc, argv, "teleop_lunabotics");

  // if (argc != 2) {
  //   cout << "Usage: " << argv[0] << " <robot-path>" << endl;
  //   quit(1);
  // }

  // string robot_path(argv[1]);
  // TeleopLunabotics teleop(robot_path);

  // signal(SIGINT, quit);

  // teleop.keyLoop();
  // quit(0);
}

// void TeleopLunabotics::keyLoop() {
//   char c;

//   double moveSpeed = 0.0;
//   double turnSpeed = 0.0;

//   puts("Reading from keyboard");
//   puts("-----------------------------------------------------------");
//   puts("Use arrow keys to move the robot, ' ' to stop, 'q' to quit.");

//   while (1) {
//     // Get the next event from the keyboard.
//     try {
//       input.readOne(&c);
//     } catch (const std::runtime_error &) {
//       perror("read():");
//       return;
//     }

//     switch (c) {
//     case KEYCODE_LEFT:
//       ROS_DEBUG("LEFT");
//       moveSpeed = 0;
//       turnSpeed -= 0.5;
//       break;
//     case KEYCODE_RIGHT:
//       ROS_DEBUG("RIGHT");
//       moveSpeed = 0;
//       turnSpeed += 0.5;
//       break;
//     case KEYCODE_UP:
//       ROS_DEBUG("UP");
//       moveSpeed += 0.5;
//       turnSpeed = 0;
//       break;
//     case KEYCODE_DOWN:
//       ROS_DEBUG("DOWN");
//       moveSpeed -= 0.5;
//       turnSpeed = 0;
//       break;
//     case KEYCODE_SPACE:
//       ROS_DEBUG("STOP");
//       moveSpeed = 0;
//       turnSpeed = 0;
//       break;
//     case KEYCODE_Q:
//       ROS_DEBUG("quit");
//       return;
//     }

//     auto nav = pair<double, double>(1.0 * moveSpeed + 1.0 * turnSpeed,
//                                     1.0 * moveSpeed - 1.0 * turnSpeed);

//     for (auto wheel : wheels_) {
//       wheel.nav(nav);
//     }
//   }

//   return;
// }
