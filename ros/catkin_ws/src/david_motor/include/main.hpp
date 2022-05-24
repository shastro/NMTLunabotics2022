// Motor controller.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#ifndef H_MAIN
#define H_MAIN

#include "ros/publisher.h"
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <ros/ros.h>

#define DEPTH_CONVERSION_FACTOR 2.9069767441860463e-06
#define DUMP_CONVERSION_FACTOR 2.9069767441860463e-06
#define DEPTH_CONVERSION_FACTOR_VEL 1.593932086646914e-10
#define DUMP_CONVERSION_FACTOR_VEL 1.593932086646914e-10 // rpm to m/s
#define LOCO_CONVERSION_FACTOR 1909.859                  // enc per rad
#define LOCO_CONVERSION_FACTOR_VEL 0.10472               // rpm to rad/s
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

struct telemetry_msg {
  double position;
  double velocity;
  double torque;
  double rms;
};

class MotorController {
private:

public:
  ros::Publisher &_telem;
  std::string _name;
  // Put the motor in Velocity mode, with the given target velocity in
  // radians per second.
  virtual void setVelocity(double vel) = 0;

  // Put the motor in Position mode, with the given target position in
  // radians.
  virtual void setPosition(double pos) = 0;

  // Get motor position (returns encoder count).
  virtual double position() = 0;

  // Get motor velocity (returns RPM).
  virtual double velocity() = 0;

  // Get measured torque (returns percentage of maximum by default).
  virtual double torque() = 0;

  // Get measured rms_level (returns percentage of maximum).
  virtual double rms() = 0;

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
  // from -1 to 1, and `multiplier` represents whether forward motion
  // is clockwise (1) or counterclockwise (-1).
  NavMotor(MotorController *m, std::string name, double position, double multiplier);

  // Rotate the motor to achieve the given linear velocity forward,
  // and angular velocity clockwise.
  void nav(double linear, double angular);
  telemetry_msg telem();
  std::string name();

private:
  // The underlying motor controller.
  std::unique_ptr<MotorController> motor_;


  // Joint name
  std::string name_;

  // The horizontal position of the motor on the robot, for use in
  // calculating rotations.
  double position_;

  // The direction to spin the motor to move forward.
  double multiplier_;
};

// Initializes the navigation motors for the robot at the given path.
std::vector<NavMotor> init_motors(std::string path, NavMotor *&augerMotor,
                                  NavMotor *&depthLMotor,
                                  NavMotor *&depthRMotor,
                                  NavMotor *&dumperLMotor,
                                  NavMotor *&dumperRMotor,
                                  ros::Publisher &telemetry_pub);


#endif // H_MAIN
