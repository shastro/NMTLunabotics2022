#include <functional>
#include <iostream>

#include <actionlib/server/simple_action_server.h>
#include <david_action/NavigationAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

using namespace std;

class NavigationAction {
  ros::NodeHandle _nh;

  actionlib::SimpleActionServer<david_action::NavigationAction> _as;
  string _action_name;

  void executeCB(const david_action::NavigationGoalConstPtr &goal) {
    // Need to publish to the navigation goal system. Then spin
    // reading from the robot base frame until it is within a
    // tolerance of the navigation goal. While we spin, publish
    // progress reports.

    {
      // Tell move_base to navigate to the goal.
      ros::Publisher goal_pub =
          _nh.advertise<geometry_msgs::Pose>("/move_base/goal", 1000);
      geometry_msgs::Pose pos = goal->target;
      goal_pub.publish(pos);
    }

    // Wait for move_base to finish.
    ros::Rate r = 5; // hz
    while (true) {
      if (_as.isPreemptRequested() || !ros::ok()) {
        cout << _action_name << ": Preempted" << endl;

        _as.setPreempted();
        return;
      }

      // TODO: Publish feedback.

      // TODO: detect conclusion.

      r.sleep();
    }

    cout << _action_name << ": Succeeded" << endl;

    // TODO: measure this error correctly.
    david_action::NavigationResult result;
    result.error = 0;
    _as.setSucceeded(result);
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
