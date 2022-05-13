#include "motor_utils.hpp"
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

  cout << "Enabling node " << node << "..." << endl;
  _enableNode();
  cout << "Enable finished." << endl;

  cout << "Setting standard units..." << endl;
  _setStandardUnits();
  cout << "Set standard units." << endl;
}

SimpleNode::~SimpleNode() {
  if (_node)
    _node->EnableReq(false);
}

SimpleNode::SimpleNode(SimpleNode &&src) {
  _mgr = src._mgr;
  _node = src._node;

  src._mgr = nullptr;
  src._node = nullptr;
}

void SimpleNode::setVel(double vel) {
  try {
    _node->Motion.MoveVelStart(vel);
  } catch (mnErr err) {
    cout << "Velocity set error: " << err.ErrorMsg << endl;
    // ignore error
  }
}

void SimpleNode::setPos(double pos) {
  try {
    _node->Motion.MovePosnStart(pos, true);
  } catch (mnErr err) {
    cout << "Position set error: " << err.ErrorMsg << endl;
    // ignore error
  }
}

int SimpleNode::type() { return _node->Info.NodeType(); }

string SimpleNode::userID() { return _node->Info.UserID.Value(); }

string SimpleNode::firmwareVersion() {
  return _node->Info.FirmwareVersion.Value();
}

int SimpleNode::serial() { return _node->Info.SerialNumber.Value(); }

string SimpleNode::model() { return _node->Info.Model.Value(); }

INode *SimpleNode::getNode() { return _node; }

void SimpleNode::_enableNode() {
  cout << "EnableReq(false)" << endl;
  _node->EnableReq(false);

  // I have no idea why this is here, but it's in the example code.
  // ~~Alex
  cout << "Delaying" << endl;
  _mgr->Delay(200);

  // Clean up node commands, status, &c. to prepare to enable the
  // node.
  cout << "Clearing commands" << endl;
  _node->Status.AlertsClear();
  _node->Motion.NodeStopClear();

  cout << "EnableReq(true)" << endl;
  _node->EnableReq(true);

  // Node should be enabled; wait around in case it takes time.
  double timeout = _mgr->TimeStampMsec() + TIME_TILL_TIMEOUT;
  cout << "Spinning for node to be ready" << endl;
  while (!_node->Motion.IsReady()) {
    if (_mgr->TimeStampMsec() > timeout) {
      throw string("Failed to enable node");
    }
  }
  cout << "Spinning finished" << endl;
}

void SimpleNode::_setStandardUnits() {
  _node->Motion.MoveWentDone();

  _node->AccUnit(INode::RPM_PER_SEC);
  _node->VelUnit(INode::RPM);
  _node->Motion.AccLimit = ACC_LIM_RPM_PER_SEC;
  _node->Motion.VelLimit = VEL_LIM_RPM;
}

SimplePort::SimplePort(SysManager *mgr, IPort *port) : _mgr(mgr), _port(port) {
  cout << "Setting up port with " << nodeCount() << " nodes..." << endl;
  for (size_t i = 0; i < nodeCount(); i++)
    _nodes.push_back(SimpleNode(_mgr, &_port->Nodes(i)));
  cout << "Finished setting up port." << endl;
}

SimplePort::~SimplePort() {
  // Need to disable every node *before* we disconnect from the port.
  _nodes.clear();

  if (_mgr)
    _mgr->PortsClose();
}

SimplePort::SimplePort(SimplePort &&src) {
  _mgr = src._mgr;
  _port = src._port;
  _nodes = move(src._nodes);

  src._mgr = nullptr;
  src._port = nullptr;
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
