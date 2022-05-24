#include <cstdio>
#include <functional>
#include <iostream>

#include <actionlib/server/simple_action_server.h>
#include <david_action/DepthAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

using namespace std;

class DepthAction {
  ros::NodeHandle _nh;

  actionlib::SimpleActionServer<david_action::DepthAction> _as;
  string _action_name;

  void executeCB(const david_action::DepthGoalConstPtr &goal) {}

public:
  DepthAction(string name)
      : _as(_nh, name, bind(&DepthAction::executeCB, this, _1), false),
        _action_name(name) {
    _as.start();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth");

  DepthAction nav("depth");
  ros::spin();
}
