// Base control.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <string>
#include <utility>
#include <vector>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <webots_ros/set_float.h>

// Direct connection to a motor of some type. This class is meant to
// abstract away the difference between a Webots motor and a physical
// motor.
//
// A `Motor' object capable of operating in two possible modes:
// velocity mode and position mode. When operating in Velocity mode,
// the motor attempts to rotate at some particular velocity and makes
// no guarantees about its position; whereas in Position mode, the
// motor moves as quickly as possible to some particular position, and
// will attempt to hold that position.
class Motor {
public:
  // Put the motor in Velocity mode, with the given target velocity in
  // radians per second.
  virtual void setVelocity(double vel) = 0;

  // Put the motor in Position mode, with the given target position in
  // radians.
  virtual void setPosition(double pos) = 0;
};

// Connect to a motor.

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

// class TeleopLunabotics {
// public:
//   TeleopLunabotics(std::string path);
//   void keyLoop();

// private:
//   ros::NodeHandle nh_;

//   // The robot's wheels.
//   std::vector<NavMotor> wheels_;

//   // The path to the base robot, e.g., "/david_28148_helios".
//   std::string robot_path_;
// };
