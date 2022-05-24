// Interface to talking to Teknic motors.

// Copyright (c) 2022 NMT Lunabotics. All rights reserved.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <pubSysCls.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "main.hpp"
#include "ros/publisher.h"
#include "teknic.hpp"
#include "teknic/motor_utils.hpp"

using namespace sFnd;
using namespace std;

// Construct a motor controller at the given path, and initialize it
// to zero velocity.
TeknicMotor::TeknicMotor(SimpleNode &node, ros::Publisher *telem,
                         std::string name)
    : _node(node), _telem(telem) {
  _name = name;
  _manager = thread([this]() { this->motor_manager(); });
}

void TeknicMotor::setVelocity(double vel) { _vel_target = vel; }

void TeknicMotor::setPosition(double pos) {
  // _node.setPos(pos);
  // That might RMS out.
  cerr << "TeknicMotor::setPosition unimplemented" << endl;
  abort();
}

// Get motor position (returns encoder count).
double TeknicMotor::position() { return _node.position(); }

// Get motor velocity (returns RPM).
double TeknicMotor::velocity() { return _node.velocity(); }

// Get measured torque (returns percentage of maximum by default).
double TeknicMotor::torque() { return _node.torque(); }

// Get measured rms_level (returns percentage of maximum).
double TeknicMotor::rms() { return _node.rms(); }

// Initializes the navigation motors for the robot.
vector<NavMotor> init_motors(string path, NavMotor *&augerMotor,
                             NavMotor *&depthLMotor, NavMotor *&depthRMotor,
                             NavMotor *&dumperLMotor, NavMotor *&dumperRMotor,
                             ros::Publisher &telemetry_pub) {
  vector<SimplePort> ports = SimplePort::getPorts();

  // This leaks `ports[0]` on purpose, because the `SimplePort` has to
  // be alive for the `SimpleNode`s to function. Ideally we would
  // change the return type here to be some type that keeps the
  // `SimplePort` alive, but I don't have time.
  SimplePort *port = new SimplePort(move(ports[0]));

  vector<SimpleNode> &nodes = port->getNodes();
  vector<NavMotor> motors;

  // And this leaks TeknicMotor objects. Cry about it.
  motors.push_back(NavMotor(new TeknicMotor(nodes.at(MotorIdent::LocomotionL),
                                            &telemetry_pub, "loco_left"),
                            "loco_left", -1, -1));
  motors.push_back(NavMotor(new TeknicMotor(nodes.at(MotorIdent::LocomotionR),
                                            &telemetry_pub, "loco_right"),
                            "loco_right", 1, 1));

  augerMotor = new NavMotor(new TeknicMotor(nodes.at(MotorIdent::Auger),
                                            &telemetry_pub, "auger_rotation"),
                            "auger_rotation", 0, 1);
  depthLMotor = new NavMotor(
      new TeknicMotor(nodes.at(MotorIdent::DepthL), &telemetry_pub, "L_depth"),
      "L_depth", 0, 1);
  depthRMotor = new NavMotor(
      new TeknicMotor(nodes.at(MotorIdent::DepthR), &telemetry_pub, "R_depth"),
      "R_depth", 0, 1);
  dumperLMotor = new NavMotor(
      new TeknicMotor(nodes.at(MotorIdent::DumpL), &telemetry_pub, "left_dump"),
      "left_dump", 0, 1);
  dumperRMotor = new NavMotor(new TeknicMotor(nodes.at(MotorIdent::DumpR),
                                              &telemetry_pub, "right_dump"),
                              "right_dump", 0, 1);

  return motors;
}

void TeknicMotor::motor_manager() {
  while (true) {
    // double rms = _node.rms();
    // double max_rms = 1;

    // // Use epsilon ratios of 1000 rpm to serve as the assumed maximum
    // // RPM, if we're not moving (i.e., _vel_current and rms are both
    // // zero).
    // double rpm_per_rms = (_vel_current + 0.001) / (rms + 0.000001);
    // double max_rpm = max_rms * rpm_per_rms;

    // _vel_current = min((double)_vel_target, max_rpm);
    // _node.setVel(_vel_current);

    // that's not working, and I'm too tired to figure out why.
    _node.setVel(_vel_target);

    if (_name == "loco_left") {
      sensor_msgs::JointState msg;
      msg.name = {_name};
      msg.position = {_node.position() * LOCO_CONVERSION_FACTOR /
                      (double)10}; // encoder count
      msg.velocity = {_node.velocity() * LOCO_CONVERSION_FACTOR_VEL /
                      (double)10}; // velocity
      msg.effort = {_node.rms()};
      _telem->publish(msg);

    } else if (_name == "loco_right") {
      sensor_msgs::JointState msg;
      msg.name = {_name};
      msg.position = {_node.position() * LOCO_CONVERSION_FACTOR /
                      (double)10}; // encoder count
      msg.velocity = {_node.velocity() * LOCO_CONVERSION_FACTOR_VEL /
                      (double)10}; // velocity
      msg.effort = {_node.rms()};
      _telem->publish(msg);

    } else if (_name == "auger_rotation") {
      sensor_msgs::JointState msg;
      msg.name = {_name};
      msg.position = {_node.position() * LOCO_CONVERSION_FACTOR /
                      (double)50}; // encoder count
      msg.velocity = {_node.velocity() * LOCO_CONVERSION_FACTOR_VEL /
                      (double)50}; // velocity
      msg.effort = {_node.rms()};
      _telem->publish(msg);

    } else if (_name == "L_depth") {
      sensor_msgs::JointState msg;
      msg.name = {_name};
      msg.position = {_node.position() *
                      DEPTH_CONVERSION_FACTOR}; // encoder count
      msg.velocity = {_node.velocity() *
                      DEPTH_CONVERSION_FACTOR_VEL}; // velocity
      msg.effort = {_node.rms()};
      _telem->publish(msg);

    } else if (_name == "R_depth") {
      sensor_msgs::JointState msg;
      msg.name = {_name};
      msg.position = {_node.position() *
                      DEPTH_CONVERSION_FACTOR}; // encoder count
      msg.velocity = {_node.velocity() *
                      DEPTH_CONVERSION_FACTOR_VEL}; // velocity
      msg.effort = {_node.rms()};
      _telem->publish(msg);

    } else if (_name == "left_dump") {
      sensor_msgs::JointState msg;
      msg.name = {_name};
      msg.position = {_node.position() *
                      DUMP_CONVERSION_FACTOR}; // encoder count
      msg.velocity = {_node.velocity() *
                      DUMP_CONVERSION_FACTOR_VEL}; // velocity
      msg.effort = {_node.rms()};
      _telem->publish(msg);

    } else if (_name == "right_dump") {
      sensor_msgs::JointState msg;
      msg.name = {_name};
      msg.position = {_node.position() *
                      DUMP_CONVERSION_FACTOR}; // encoder count
      msg.velocity = {_node.velocity() *
                      DUMP_CONVERSION_FACTOR_VEL}; // velocity
      msg.effort = {_node.rms()};
      _telem->publish(msg);
    }

    std::this_thread::sleep_for(
        std::chrono::microseconds(1000000 / MANAGER_RESOLUTION));
  }
}
