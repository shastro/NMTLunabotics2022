// ROS controller.

// Copyright (c) 2021 NMT Lunabotics.
// All rights reserved.

#include <string>
#include <utility>

#include <ros/ros.h>
#include <webots_ros/set_float.h>

// Reading keycodes from the terminal is OS-dependent; this class
// implements such a process for Linux and Windows.
class KeyboardReader {
public:
  KeyboardReader();
  void readOne(char *c);
  void shutdown();

private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

// Direct connection to a motor.
//
// A `Motor' object is designed to operate in two possible modes:
// velocity mode, and position mode. When operating in Velocity mode,
// the motor attempts to rotate at some particular velocity and makes
// no guarantees about its position; whereas in Position mode, the
// motor moves as quickly as possible to some particular position, and
// will attempt to hold that position.
class Motor {
public:
  // Initialize a new Motor with the given path. By default, the Motor
  // is in Position mode, locked to 0 radians.
  Motor(std::string Path);

  // Put the motor in Velocity mode, with the given target velocity in
  // radians per second.
  void setVelocity(double vel);

  // Put the motor in Position mode, with the given target position in
  // radians.
  void setPosition(double pos);

private:
  // Send ROS the current value of `pos_' and `vel_'.
  void update();

  // The target velocity and position. If we're in Velocity mode, then
  // `pos_' is infinity. Otherwise, the value of `vel_' is ignored.
  double pos_;
  double vel_;

  // The ROS path of the wheel position and velocity topics,
  // respectively.
  std::string pos_path_;
  std::string vel_path_;
};

// A Motor, coupled with some information on how that motor's
// rotations relate to the actions the robot is trying to take.
//
// At the moment, the algorithm is as follows: the velocity of each
// motor is defined to be equal to the dot product of two
// two-dimensional vectors: the "motor vector", which varies per
// motor, and the "navigation vector", which is global for the entire
// robot.
//
// In general, the navigation vector's components represent how fast
// the left and right sides of the robot should be moving; for
// example, (0, 0) is stationary, (1, 1) is moving uniformly forward,
// and (1, -1) is rotating clockwise on the spot. Correspondingly,
// each motor vector's components represent how far that motor is to
// the left or right on the robot, and the two components of the
// vector should add up to 1. For example, (1, 0) is on the left edge
// of the robot, (0, 1) is on the right edge of the robot, and (2, -1)
// is one unit to the left of the left edge of the robot.
class NavMotor {
public:
  NavMotor(Motor m, std::pair<double, double> mot_vec);

  // Set the motor's velocity to track the given navigation vector.
  void nav(std::pair<double, double> nav_vec);

private:
  Motor motor_;
  std::pair<double, double> mot_vec_;
};

class TeleopLunabotics {
public:
  TeleopLunabotics(std::string path);
  void keyLoop();

private:
  ros::NodeHandle nh_;

  // The robot's four wheels.
  NavMotor back_left_, back_right_, front_left_, front_right_;

  // The path to the base robot, e.g., "/david_28148_helios".
  std::string robot_path_;
};
