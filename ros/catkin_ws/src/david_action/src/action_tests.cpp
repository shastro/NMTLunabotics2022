#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <david_action/DepthAction.h>
#include <david_action/PitchAction.h>
#include <ros/ros.h>

using namespace david_action;
using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_tests");
  vector<string> args(argv, argv + argc);

  if (args.at(1) == "depth") {
    cerr << "todo" << endl;
  } else if (args.at(1) == "pitch") {
    actionlib::SimpleActionClient<PitchAction> ac("pitch", true);
    ac.waitForServer();
    PitchGoal goal;

    if (args.at(2) == "home") {
      goal.goal_state = 0;
    } else if (args.at(2) == "extend") {
      goal.goal_state = 1;
    } else if (args.at(2) == "retract") {
      goal.goal_state = 2;
    } else if (args.at(3) == "half_extend") {
      goal.goal_state = 3;
    } else {
      cout << "Unknown goal state " << args.at(2);
      return EXIT_FAILURE;
    }

    cout << "Sending goal" << endl;
    ac.sendGoal(goal);
    cout << "Goal sent" << endl;
    bool finished_before_timeout = ac.waitForResult(ros::Duration(55.0));
    cout << "Finished = " << finished_before_timeout << endl;
  } else {
    cout << "Unknown test " << args.at(1) << endl;;
  }
}
