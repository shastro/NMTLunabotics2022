#include "8Bitdoh.hpp"
#include "joystick.hh"
#include "pubSysCls.h"
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

#define ACC_LIM_RPM_PER_SEC 500
#define VEL_LIM_RPM 1000
#define MOVE_DISTANCE_CNTS 5000
#define NUM_MOVES 5
#define TIME_TILL_TIMEOUT 10000 // The timeout used for homing(ms)

typedef size_t MotorID;

// Motor identifier constants. TODO: Combine this and MotorID.
class MotorIdent {
public:
  static const MotorID Auger = 0;
  static const MotorID DumpL = 1;
  static const MotorID DepthL = 2;
  static const MotorID LocomotionL = 3;
  static const MotorID LocomotionR = 4;
  static const MotorID DepthR = 5;
  static const MotorID DumpR = 6;
};

// A command associated with pressing a button.
struct ButtonCommand {
  // Create a new ButtonCommand, in which the list of `motors' all
  // move in direction `direction'.
  ButtonCommand(std::vector<MotorID> _motors, int _direction);

  std::vector<MotorID> motors;
  int direction;
};

// A command associated with moving an axis.
struct AxisCommand {
  // Create a new AxisCommand, in which the list of `motors' all move
  // in direction `direction'. An axis position of `basis' is treated
  // as zero (no motion).
  AxisCommand(std::vector<MotorID> _motors, int _basis, int _direction);

  std::vector<MotorID> motors;
  int basis;
  int direction;
};

// ---- motor_utils.cpp ----

// Send message and wait for newline.
void msgUser(const char *msg);

// Set the velocity of a node on the port.
void setNodeVel(sFnd::SysManager *mgr, sFnd::IPort *port, MotorID nodeNum,
                double vel);

// Set the velocity of every motor on the port.
void moveNodesFixed(sFnd::SysManager *myMgr, sFnd::IPort *myPort, double goal);

// Set the position of a node on the port.
void setNodePos(sFnd::SysManager *mgr, sFnd::IPort *port, MotorID nodeNum,
                double pos);
