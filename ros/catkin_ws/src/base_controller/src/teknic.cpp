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

using namespace sFnd;
using namespace std;

#define TODO abort()

// Construct a motor controller at the given path, and initialize it
// to zero velocity.
TeknicMotor::TeknicMotor() { TODO; }

void TeknicMotor::setVelocity(double vel) { TODO; }

void TeknicMotor::setPosition(double pos) { TODO; }

// Initializes the navigation motors for the robot.
vector<NavMotor> init_motors(string path) {
  vector<NavMotor> motors;

  SysManager *mgr = SysManager::Instance();
  try {
    size_t port_count = setup_ports(mgr);

    for (size_t i = 0; i < port_count; i++) {
      IPort &port = mgr->Ports(i);
      cout << "Port " << port.netNumber() << ": state = " << port.openState()
           << ", nodes = << " << port.NodeCount() << endl;
      for (size_t node_num = 0; node_num < port.NodeCount(); node_num++) {
        INode &node = port.Nodes(node_num);

        // Ensure node is disabled before loading config file.
        node.EnableReq(false);

        mgr->Delay(200);

        cout << "Node " << node_num << ": type = " << node.Info.NodeType()
             << endl
             << "UserID: " << node.Info.UserID.Value() << endl
             << "FW version: " << node.Info.FirmwareVersion.Value() << endl
             << "Serial: " << node.Info.SerialNumber.Value() << endl
             << "Model: " << node.Info.Model.Value() << endl;

        // Enable the node.
        node.Status.AlertsClear();
        node.Motion.NodeStopClear();
        node.EnableReq(true);

        cout << "Node " << node_num << " enabled." << endl;

        double timeout = mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;
        while (!node.Motion.IsReady()) {
          if (mgr->TimeStampMsec() > timeout) {
            cerr << "Timed out waiting for node " << node_num << " to enable."
                 << endl;
            abort();
          }

          usleep(POLL_INTERVAL * 1000);
        }

        // Would home motors here; but according to documentation
        // (IHoming description):
        // > Homing is required to use soft limits and establish the
        // > node's number space for absolute type moves.
        // Given that we're not using soft limits or absolute type
        // moves, we don't care about homing.
        TODO;
      }
      TODO;
    }
    TODO;
  } catch (mnErr &err) {
    TODO;
  }

  return motors;
}

// Sets up the ports on the SysManager; returns the number of ports
// initialized.
static size_t setup_ports(SysManager *mgr) {
  vector<string> com_hub_ports;
  SysManager::FindComHubPorts(com_hub_ports);
  cout << "Debug: found " << com_hub_ports.size() << " hubs" << endl;

  if (com_hub_ports.size() == 0) {
    cerr << "No SC hub port found." << endl;
    abort();
  }

  int port_count = min(com_hub_ports.size(), NET_CONTROLLER_MAX);
  for (int i = 0; i < port_count; i++) {
    mgr->ComHubPort(i, com_hub_ports[i].c_str());
  }

  mgr->PortsOpen(port_count);

  return port_count;
}
