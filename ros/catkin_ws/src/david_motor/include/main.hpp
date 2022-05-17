// Motor controller.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#ifndef H_MAIN
#define H_MAIN

#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <ros/ros.h>

// Direct connection to a motor of some type. This class is meant to
// abstract away the difference between a Webots motor and a physical
// motor.
//
// A `MotorController' object capable of operating in two possible
// modes: velocity mode and position mode. When operating in Velocity
// mode, the motor attempts to rotate at some particular velocity and
// makes no guarantees about its position; whereas in Position mode,
// the motor moves as quickly as possible to some particular position,
// and will attempt to hold that position.
class MotorController {
public:
  // Put the motor in Velocity mode, with the given target velocity in
  // radians per second.
  virtual void setVelocity(double vel) = 0;

  // Put the motor in Position mode, with the given target position in
  // radians.
  virtual void setPosition(double pos) = 0;

  virtual ~MotorController(){};
};

// A Motor used for navigation. Specifically, a motor that contains
// information about where it is physically located on the motor, and
// that can thus be fed commands regarding the robot's overall motion
// to rotate appropriately.
class NavMotor {
public:
  // Construct a new NavMotor, taking ownership of the `Motor`.
  // `position` is the horizontal position of the motor on the robot,
  // from -1 to 1.
  NavMotor(MotorController *m, double position);

  // Rotate the motor to achieve the given linear velocity forward,
  // and angular velocity clockwise.
  void nav(double linear, double angular);

private:
  std::unique_ptr<MotorController> motor_;
  double position_;
};

// Initializes the navigation motors for the robot at the given path.
std::vector<NavMotor> init_motors(std::string path, NavMotor *&augerMotor,
                                  NavMotor *&depthLMotor,
                                  NavMotor *&depthRMotor,
                                  NavMotor *&dumperLMotor,
                                  NavMotor *&dumperRMotor);

#endif // H_MAIN
