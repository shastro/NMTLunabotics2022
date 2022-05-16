// Webots-specific motor controller implementation details.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#ifndef H_WEBOTS
#define H_WEBOTS

#include <string>

#include "main.hpp"

// Marker for undefined position.
#define UND_POS (std::numeric_limits<double>::infinity())

class WebotsMotor : public MotorController {
public:
  // Initialize a new WebotsMotor with the given path. By default, the
  // Motor is in Velocity mode, with 0 radians per second.
  WebotsMotor(std::string Path);

  // Motor controller functions.
  void setVelocity(double vel);
  void setPosition(double pos);

private:
  // Send ROS the current value of `pos_` and `vel_`.
  void update();

  // The target velocity and position. If we're in Velocity mode, then
  // `pos_` is UND_POS and `vel_` is the target velocity in radians
  // per second; otherwise, `pos_` is the target position in radians
  // and `vel_` is ignored.
  double pos_;
  double vel_;

  // The ROS path of the wheel position and velocity topics,
  // respectively.
  std::string pos_path_;
  std::string vel_path_;
};

#endif  // H_WEBOTS
