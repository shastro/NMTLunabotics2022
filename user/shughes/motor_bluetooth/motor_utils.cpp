#include "single_motor_test.hpp"

using namespace sFnd;

// Send message and wait for newline
void msgUser(const char *msg) {
  std::cout << msg;
  getchar();
}
// Will move all motors on the port a fixed distance
void moveNodesFixed(SysManager *myMgr, IPort *myPort, double goal) {
  for (size_t iNode = 0; iNode < myPort->NodeCount(); iNode++) {
    // Create a shortcut reference for a node
    INode &theNode = myPort->Nodes(iNode);

    theNode.Motion.MoveWentDone(); // Clear the rising edge Move done register

    theNode.AccUnit(
        INode::RPM_PER_SEC);     // Set the units for Acceleration to RPM/SEC
    theNode.VelUnit(INode::RPM); // Set the units for Velocity to RPM
    theNode.Motion.AccLimit =
        ACC_LIM_RPM_PER_SEC;               // Set Acceleration Limit (RPM/Sec)
    theNode.Motion.VelLimit = VEL_LIM_RPM; // Set Velocity Limit (RPM)

    printf("Moving Node \t%zi \n", iNode);
    theNode.Motion.MoveVelStart(goal);
    // theNode.Motion.MovePosnStart(MOVE_DISTANCE_CNTS);			//Execute 10000
    // encoder count move printf("%f estimated time.\n",
    // theNode.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTS)); double timeout
    // = myMgr->TimeStampMsec() +
    // theNode.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTS) + 100;			//define
    // a timeout in case the node is unable to enable

    // 	while (!theNode.Motion.MoveIsDone()) {
    // 		if (myMgr->TimeStampMsec() > timeout) {
    // 			printf("Error: Timed out waiting for move to
    // complete\n"); 			msgUser("Press any key to continue."); //pause so the user
    // can see the error message; waits for user to press a key 			abort();
    // 		}
    // 	}
    // 	printf("Node \t%zi Move Done\n", iNode);
  } // for each node
}
