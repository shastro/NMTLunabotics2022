#include <functional>
#include <iostream>

#include <actionlib/server/simple_action_server.h>
#include <david_action/NavigationAction.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

using namespace std;

class NavigationAction {
  ros::NodeHandle _nh;

  actionlib::SimpleActionServer<david_action::NavigationAction> _as;
  string _action_name;

  void executeCB(const david_action::NavigationGoalConstPtr &goal) {
    const david_action::NavigationGoal &pgoal = *goal;
  }

public:
  NavigationAction(string name)
      : _as(_nh, name, bind(&NavigationAction::executeCB, this, _1), false),
        _action_name(name) {
    _as.start();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "navigation");

  NavigationAction nav("navigation");
  ros::spin();
}
