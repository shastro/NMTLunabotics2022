// Interface to talking to Webots motors.

// Copyright (c) 2022 NMT Lunabotics. All rights reserved.

#include <ros/ros.h>
#include <webots_ros/set_float.h>

#include <string>

#include "main.hpp"

using std::string;
using std::pair;

// Construct a motor controller at the given path, and initialize it
// to zero velocity.
Motor::Motor(string path) : pos_(UND_POS), vel_(0) {
  // These are standard for Webots -- will need to change for physical
  // motors.
  pos_path_ = path + "/set_position";
  vel_path_ = path + "/set_velocity";

  update();
}

void Motor::update() {
  webots_ros::set_float srv_pos;
  srv_pos.request.value = pos_;
  ros::service::call(pos_path_, srv_pos);

  webots_ros::set_float srv_vel;
  srv_vel.request.value = vel_;
  ros::service::call(vel_path_, srv_vel);
}

void Motor::setVelocity(double vel) {
  // Avoid sending duplicate signals to ROS if we can avoid it.
  if (pos_ == UND_POS && vel_ == vel) {
    return;
  }

  pos_ = UND_POS;
  vel_ = vel;

  update();
}

void Motor::setPosition(double pos) {
  if (pos_ == pos) {
    return;
  }

  pos_ = pos;
  // The value of `vel_' will be ignored anyway, don't bother fiddling
  // with it.

  update();
}

NavMotor::NavMotor(Motor m, pair<double, double> mot_vec)
    : motor_(m), mot_vec_(mot_vec) {}

void NavMotor::nav(pair<double, double> nav_vec) {
  auto dot = mot_vec_.first * nav_vec.first + mot_vec_.second * nav_vec.second;
  motor_.setVelocity(dot);
}
