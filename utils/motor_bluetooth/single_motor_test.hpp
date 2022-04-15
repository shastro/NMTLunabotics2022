#include <iostream>
#include <stdio.h>
#include <string>
#include "pubSysCls.h"
#include "8Bitdoh.hpp"
#include "joystick.hh"

#define ACC_LIM_RPM_PER_SEC 500
#define VEL_LIM_RPM 1000
#define MOVE_DISTANCE_CNTS 5000
#define NUM_MOVES 5
#define TIME_TILL_TIMEOUT 10000 // The timeout used for homing(ms)


void msgUser(const char *msg);


void moveNodesFixed(sFnd::SysManager *myMgr, sFnd::IPort *myPort, double goal);
