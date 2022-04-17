#include "8Bitdoh.hpp"
#include "joystick.hh"
#include "pubSysCls.h"
#include <iostream>
#include <stdio.h>
#include <string>

#define ACC_LIM_RPM_PER_SEC 500
#define VEL_LIM_RPM 1000
#define MOVE_DISTANCE_CNTS 5000
#define NUM_MOVES 5
#define TIME_TILL_TIMEOUT 10000 // The timeout used for homing(ms)

typedef size_t MotorID;

// ---- motor_utils.cpp ----

// Send message and wait for newline.
void msgUser(const char *msg);

// Set the velocity of a node on the port.
void setNodeVel(sFnd::SysManager *mgr, sFnd::IPort *port, MotorID nodeNum,
                double vel);

// Set the velocity of every motor on the port.
void moveNodesFixed(sFnd::SysManager *myMgr, sFnd::IPort *myPort, double goal);
