#include "single_motor_test.hpp"
#include <string>
#include <vector>

using namespace sFnd;
using namespace std;

// Send message and wait for newline.
void msgUser(const char *msg) {
  std::cout << msg;
  getchar();
}

SimpleNode::SimpleNode(SysManager *mgr, INode *node) {
  _mgr = mgr;
  _node = node;

  _enableNode();
  _setStandardUnits();
}

SimpleNode::~SimpleNode() { _node->EnableReq(false); }

void SimpleNode::setVel(double vel) { _node->Motion.MoveVelStart(vel); }

void SimpleNode::setPos(double pos) { _node->Motion.MovePosnStart(pos, true); }

int SimpleNode::type() { return _node->Info.NodeType(); }

string SimpleNode::userID() { return _node->Info.UserID.Value(); }

string SimpleNode::firmwareVersion() {
  return _node->Info.FirmwareVersion.Value();
}

int SimpleNode::serial() { return _node->Info.SerialNumber.Value(); }

string SimpleNode::model() { return _node->Info.Model.Value(); }

INode *SimpleNode::getNode() { return _node; }

void SimpleNode::_enableNode() {
  _node->EnableReq(false);

  // I have no idea why this is here, but it's in the example code.
  // ~~Alex
  _mgr->Delay(200);

  // Clean up node commands, status, &c. to prepare to enable the
  // node.
  _node->Status.AlertsClear();
  _node->Motion.NodeStopClear();

  _node->EnableReq(false);

  // Node should be enabled; wait around in case it takes time.
  double timeout = _mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;
  while (!_node->Motion.IsReady()) {
    if (_mgr->TimeStampMsec() > timeout) {
      throw "Failed to enable node";
    }
  }
}

void SimpleNode::_setStandardUnits() {
  _node->Motion.MoveWentDone();

  _node->AccUnit(INode::RPM_PER_SEC);
  _node->VelUnit(INode::RPM);
  _node->Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
  _node->Motion.VelLimit = VEL_LIM_RPM;
}

SimplePort::SimplePort(SysManager *mgr, IPort *port) : _mgr(mgr), _port(port) {
  for (size_t i = 0; i < nodeCount(); i++)
    _nodes.push_back(SimpleNode(_mgr, &_port->Nodes(i)));
}

SimplePort::~SimplePort() {
  _nodes = {};
  _mgr->PortsClose();
}

vector<SimplePort> SimplePort::getPorts() {
  size_t portCount = 0;
  vector<string> comHubPorts;

  SysManager::FindComHubPorts(comHubPorts);
  SysManager *mgr = SysManager::Instance();
  for (; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX;
       portCount++)
    mgr->ComHubPort(portCount, comHubPorts[portCount].c_str());

  mgr->PortsOpen(portCount);

  vector<SimplePort> ports;
  for (size_t i = 0; i < portCount; i++)
    ports.push_back(SimplePort(mgr, &mgr->Ports(i)));

  return ports;
}

int SimplePort::netNumber() { return _port->NetNumber(); }

openStates SimplePort::openState() { return _port->OpenState(); }

size_t SimplePort::nodeCount() { return _port->NodeCount(); }

vector<SimpleNode> &SimplePort::getNodes() { return _nodes; }

IPort *SimplePort::getPort() { return _port; }
