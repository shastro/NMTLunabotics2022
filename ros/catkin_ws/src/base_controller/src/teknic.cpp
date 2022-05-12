// Interface to talking to Teknic motors.

// Copyright (c) 2022 NMT Lunabotics. All rights reserved.

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <pubSysCls.h>
#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <vector>

#include "main.hpp"
#include "teknic.hpp"
#include "teknic/motor_utils.hpp"

using namespace sFnd;
using namespace std;

// Construct a motor controller at the given path, and initialize it
// to zero velocity.
TeknicMotor::TeknicMotor(SimpleNode &node) : _node(node) {}

void TeknicMotor::setVelocity(double vel) { _node.setVel(vel); }

void TeknicMotor::setPosition(double pos) { _node.setPos(pos); }

// Initializes the navigation motors for the robot.
vector<NavMotor> init_motors(string path) {
  vector<SimplePort> ports = SimplePort::getPorts();

  // This leaks `ports[0]` on purpose, because the `SimplePort` has to
  // be alive for the `SimpleNode`s to function. Ideally we would
  // change the return type here to be some type that keeps the
  // `SimplePort` alive, but I don't have time.
  SimplePort *port = new SimplePort(move(ports[0]));

  vector<SimpleNode> &nodes = port->getNodes();
  vector<NavMotor> motors;

  // And this leaks TeknicMotor objects. Cry about it.
  motors.push_back(
      NavMotor(new TeknicMotor(nodes.at(MotorIdent::LocomotionL)), -1));
  motors.push_back(
      NavMotor(new TeknicMotor(nodes.at(MotorIdent::LocomotionR)), 1));

  return motors;
}


