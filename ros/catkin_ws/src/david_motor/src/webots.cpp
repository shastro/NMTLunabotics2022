// Interface to talking to Webots motors.

// Copyright (c) 2022 NMT Lunabotics. All rights reserved.

#include <ros/ros.h>
#include <webots_ros/set_float.h>

#include "main.hpp"
#include "webots.hpp"

using namespace std;

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
  std::cout << "Debug: updating position & velocity";

  webots_ros::set_float srv_pos;
  srv_pos.request.value = pos_;
  ros::service::call(pos_path_, srv_pos);

  webots_ros::set_float srv_vel;
  srv_vel.request.value = vel_;
  ros::service::call(vel_path_, srv_vel);
}

// Initializes the navigation motors for the robot.
vector<NavMotor> init_motors(string path, NavMotor *&augerMotor,
                             NavMotor *&depthLMotor, NavMotor *&depthRMotor,
                             NavMotor *&dumperLMotor, NavMotor *&dumperRMotor) {
  // This is specific to the current Webots demo robot.
  vector<NavMotor> motors;
  motors.push_back(NavMotor(new WebotsMotor(path + "/left_motor"), -0.264));
  motors.push_back(NavMotor(new WebotsMotor(path + "/right_motor"), 0.264));

  return motors;
}
