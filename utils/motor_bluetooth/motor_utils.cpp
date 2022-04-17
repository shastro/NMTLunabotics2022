#include "single_motor_test.hpp"

using namespace sFnd;

// Send message and wait for newline.
void msgUser(const char *msg) {
  std::cout << msg;
  getchar();
}

// Set the velocity of a node on the port.
void setNodeVel(SysManager *mgr, IPort *port, size_t nodeNum, double vel) {
  INode &node = port->Nodes(nodeNum);

  node.Motion.MoveWentDone();

  node.AccUnit(INode::RPM_PER_SEC);
  node.VelUnit(INode::RPM);
  node.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
  node.Motion.VelLimit = VEL_LIM_RPM;

  printf("Moving Node \t%zi \n", nodeNum);
  node.Motion.MoveVelStart(vel);
}

// Set the velocity of every motor on the port.
void moveNodesFixed(SysManager *mgr, IPort *port, double vel) {
  for (int i = 0; i < port->NodeCount(); i++) {
    setNodeVel(mgr, port, i, vel);
  }
}
