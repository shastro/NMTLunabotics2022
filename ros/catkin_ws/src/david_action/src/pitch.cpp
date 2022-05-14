#include <cstdio>
#include <functional>
#include <iostream>

#include <actionlib/server/simple_action_server.h>
#include <david_action/PitchAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <chrono>
#include <thread>

using namespace std;
enum goal {
  HOME = 0,
  EXTEND = 1,
  RETRACT = 2,
  HALF_EXTEND = 3,
};

// Robot constants



class PitchAction {
  ros::NodeHandle _nh;

  actionlib::SimpleActionServer<david_action::PitchAction> _as;
  string _action_name;

  void executeCB(const david_action::PitchGoalConstPtr &goal) {
    // Do motor stuff

    // Wait for motors
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Publish joint states
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
