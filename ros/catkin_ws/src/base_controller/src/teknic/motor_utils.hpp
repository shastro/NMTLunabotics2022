#ifndef H_MOTOR_UTILS_
#define H_MOTOR_UTILS_

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

// A Teknic node, i.e., a motor. Teknic motors are powerful but have a
// complicated interface; this class exposes just the functionality we
// need for Lunabotics, and abstracts away much of the boilerplate
// code for working with motors.
class SimpleNode {
public:
  // Wraps the INode in a SimpleNode class, enabling the node and
  // setting it to use standard units.
  SimpleNode(sFnd::SysManager *mgr, sFnd::INode *node);

  // Disables the motor before shutdown.
  ~SimpleNode();

  // Nodes are moved, not copied.
  SimpleNode(SimpleNode &src) = delete;
  SimpleNode(SimpleNode &&src);

  // Sets the motor's velocity, disregarding position.
  void setVel(double vel);

  // Sets the motor's target position. Only works on tuned, homed
  // motors.
  void setPos(double pos);

  // Gets the motor's type.
  int type();

  // Gets the motor's user ID.
  std::string userID();

  // Gets the motor's firmware version.
  std::string firmwareVersion();

  // Gets the motor's serial number.
  int serial();

  // Gets the motor's model number.
  std::string model();

  // Gets the raw underlying node object, for more advanced
  // operations.
  sFnd::INode *getNode();


  // Get motor position (returns encoder count)
  double position();

  // Get motor velocity (returns RPM by default)
  double velocity();

  // Get measured torque (returns percentage of maximum by default)
  double torque();

private:
  // Attempts to turn on the motor; throws an exception if this fails.
  void _enableNode();

  // Sets the motor to use the standard units (revolutions per minute,
  // etc.).
  void _setStandardUnits();

  sFnd::SysManager *_mgr;
  sFnd::INode *_node;
};

class SimplePort {
public:
  // Wraps the IPort in a SimplePort class, initializing all the nodes
  // (motors) on the port.
  SimplePort(sFnd::SysManager *mgr, sFnd::IPort *port);

  // Disables all motors and disconnects from the port.
  ~SimplePort();

  // Ports are moved, not copied.
  SimplePort(SimplePort &src) = delete;
  SimplePort(SimplePort &&src);

  // Gets the list of every port (Teknic board) connected to the host
  // machine.
  static std::vector<SimplePort> getPorts();

  // Gets the network number of this port.
  int netNumber();

  // Gets whether this port is open, flashing, etc.
  openStates openState();

  // Gets the number of nodes (motors) attached to this port.
  size_t nodeCount();

  // Gets the list of nodes attached to this port.
  std::vector<SimpleNode> &getNodes();

  // Gets the raw underlying port object, for more advanced
  // operations.
  sFnd::IPort *getPort();

private:
  sFnd::SysManager *_mgr;
  sFnd::IPort *_port;
  std::vector<SimpleNode> _nodes;
};

#endif // H_MOTOR_UTILS_
