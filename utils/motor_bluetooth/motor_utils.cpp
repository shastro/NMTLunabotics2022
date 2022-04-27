#include "single_motor_test.hpp"

using namespace sFnd;

static void setDefaultConfig(INode &node);

// Send message and wait for newline.
void msgUser(const char *msg) {
  std::cout << msg;
  getchar();
}

// Set the velocity of a node on the port.
void setNodeVel(SysManager *mgr, IPort *port, MotorID nodeNum, double vel) {
  INode &node = port->Nodes(nodeNum);

  setDefaultConfig(node);

  printf("Moving Node \t%zi \n", nodeNum);
  node.Motion.MoveVelStart(vel);
}

// Set the velocity of every motor on the port.
void setAllNodeVel(SysManager *mgr, IPort *port, double vel) {
  for (int i = 0; i < port->NodeCount(); i++) {
    setNodeVel(mgr, port, i, vel);
  }
}

// Set the absolute position of a node on the port.
void setNodePos(SysManager *mgr, IPort *port, MotorID nodeNum, double pos) {
  INode &node = port->Nodes(nodeNum);

  setDefaultConfig(node);

  printf("Moving Node \t%zi \n", nodeNum);
  node.Motion.MovePosnStart(pos, true);
}

static void setDefaultConfig(INode &node) {
  node.Motion.MoveWentDone();

  node.AccUnit(INode::RPM_PER_SEC);
  node.VelUnit(INode::RPM);
  node.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
  node.Motion.VelLimit = VEL_LIM_RPM;
}
