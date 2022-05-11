#include <cstdio>
#include <functional>
#include <iostream>

#include <actionlib/server/simple_action_server.h>
#include <david_action/PitchAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

using namespace std;

class PitchAction {
  ros::NodeHandle _nh;

  actionlib::SimpleActionServer<david_action::PitchAction> _as;
  string _action_name;

  void executeCB(const david_action::PitchGoalConstPtr &goal) {
  }

public:
  PitchAction(string name)
      : _as(_nh, name, bind(&PitchAction::executeCB, this, _1), false),
        _action_name(name) {
    _as.start();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pitch");

  PitchAction nav("pitch");
  ros::spin();
}
