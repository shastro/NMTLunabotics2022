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
  // move with velocity `velocity` with the sign indicating direction.
  ButtonCommand(std::vector<MotorID> _motors, int _velocity);

  std::vector<MotorID> motors;
  int velocity;
};

// A command associated with moving an axis.
struct AxisCommand {
  // Create a new AxisCommand, in which the list of `motors' all move
  // in direction `direction'. An axis position of `basis' is treated
  // as zero (no motion).
  AxisCommand(std::vector<MotorID> _motors, int _basis, int _velocity);

  std::vector<MotorID> motors;
  int basis;
  int velocity;
};

// ---- motor_utils.cpp ----

// Send message and wait for newline.
void msgUser(const char *msg);

class SimpleNode {
public:
  SimpleNode(sFnd::SysManager *mgr, sFnd::INode *node);
  ~SimpleNode();

  void setVel(double vel);
  void setPos(double pos);

  int type();
  std::string userID();
  std::string firmwareVersion();
  int serial();
  std::string model();

private:
  void _enableNode();
  void _setStandardUnits();

  sFnd::SysManager *_mgr;
  sFnd::INode *_node;
};

class SimplePort {
public:
  SimplePort(sFnd::SysManager *mgr, sFnd::IPort *port);
  ~SimplePort();

  static std::vector<SimplePort> getPorts();

  int netNumber();
  int openState();
  size_t nodeCount();

  std::vector<SimpleNode> &getNodes();

private:
  sFnd::SysManager *_mgr;
  sFnd::IPort *_port;
  std::vector<SimpleNode> _nodes;
};
