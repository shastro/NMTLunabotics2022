// Interface to talking to Teknic motors.

// Copyright (c) 2022 NMT Lunabotics. All rights reserved.

#include <cstdlib>
#include <iostream>
#include <pubSysCls.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "main.hpp"
#include "teknic.hpp"

using namespace sFnd;
using namespace std;

#define TODO abort()

// Construct a motor controller at the given path, and initialize it
// to zero velocity.
TeknicMotor::TeknicMotor() {
  TODO;
}

void TeknicMotor::setVelocity(double vel) {
  TODO;
}

void TeknicMotor::setPosition(double pos) {
  TODO;
}

// Initializes the navigation motors for the robot.
vector<NavMotor> init_motors(string path) {
  vector<NavMotor> motors;

  SysManager *mgr = SysManager::Instance();
  try {
    vector<string> com_hub_ports;
    SysManager::FindComHubPorts(com_hub_ports);
    cout << "Debug: found " << com_hub_ports.size() << " hubs" << endl;

    if (com_hub_ports.size() == 0) {
      cerr << "No SC hub port found." << endl;
      abort();
    }

    for (int port_count = 0;
         port_count < com_hub_ports.size() && port_count < NET_CONTROLLER_MAX;
         port_count++) {
      mgr->ComHubPort(port_count, com_hub_ports[port_count].c_str());
    }

    TODO;
  } catch (mnErr &err) {
    TODO;
  }

  return motors;
}
