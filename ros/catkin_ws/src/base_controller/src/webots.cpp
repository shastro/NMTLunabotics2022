// Interface to talking to Webots motors.

// Copyright (c) 2022 NMT Lunabotics. All rights reserved.

#include <ros/ros.h>
#include <webots_ros/set_float.h>

#include "main.hpp"
#include "webots.hpp"

using std::pair;
using std::string;

// Construct a motor controller at the given path, and initialize it
// to zero velocity.
WebotsMotor::WebotsMotor(string path) : pos_(UND_POS), vel_(0) {
  pos_path_ = path + "/set_position";
  vel_path_ = path + "/set_velocity";

  update();
}

void WebotsMotor::setVelocity(double vel) {
  // Avoid sending duplicate signals to ROS if we can avoid it.
  if (pos_ == UND_POS && vel_ == vel) {
    return;
  }

  pos_ = UND_POS;
  vel_ = vel;

  update();
}

void WebotsMotor::setPosition(double pos) {
  if (pos_ == pos) {
    return;
  }

  pos_ = pos;
  // The value of `vel_' will be ignored anyway, don't bother fiddling
  // with it.

  update();
}

// Send the motor's `pos_` and `vel_` to Webots.
void WebotsMotor::update() {
  webots_ros::set_float srv_pos;
  srv_pos.request.value = pos_;
  ros::service::call(pos_path_, srv_pos);

  webots_ros::set_float srv_vel;
  srv_vel.request.value = vel_;
  ros::service::call(vel_path_, srv_vel);
}

// Initializes the navigation motors for the robot.
std::vector<NavMotor> init_motors(string path) {
  // This is specific to the current Webots demo robot.
  std::vector<NavMotor> motors;
  motors.push_back(NavMotor(new WebotsMotor(path + "/motor1"), -1.0));
  motors.push_back(NavMotor(new WebotsMotor(path + "/motor2"), -1.0));
  motors.push_back(NavMotor(new WebotsMotor(path + "/motor3"), 1.0));
  motors.push_back(NavMotor(new WebotsMotor(path + "/motor4"), 1.0));
  motors.push_back(NavMotor(new WebotsMotor(path + "/motor5"), -1.0));
  motors.push_back(NavMotor(new WebotsMotor(path + "/motor6"), -1.0));
  motors.push_back(NavMotor(new WebotsMotor(path + "/motor7"), 1.0));
  motors.push_back(NavMotor(new WebotsMotor(path + "/motor8"), 1.0));

  return motors;
}
