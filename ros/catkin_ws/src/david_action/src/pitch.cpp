#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <david_action/PitchAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include "gpio_lib/rpi_gpio.hpp"

using namespace std;

enum class Goal {
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

  GPIOOut _home;
  GPIOOut _extend;
  GPIOOut _retract;
  GPIOOut _half_extend;

  void executeCB(const david_action::PitchGoalConstPtr &goal) {
    // Do motor stuff
    GPIOOut *pin;
    switch ((Goal)goal->goal_state) {
    case Goal::HOME:
      pin = &_home;
      break;
    case Goal::EXTEND:
      pin = &_extend;
      break;
    case Goal::RETRACT:
      pin = &_retract;
      break;
    case Goal::HALF_EXTEND:
      pin = &_half_extend;
      break;
    default:
      cerr << "Warning: PitchAction: ignoring invalid goal state"
           << goal->goal_state << endl;
      return;
    }

    // Pulse the control pin quickly.
    pin->set(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pin->set(false);

    // Wait for motors
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Publish joint states
  }

public:
  PitchAction(string name)
      : _as(_nh, name, bind(&PitchAction::executeCB, this, _1), false),
        _action_name(name),
        // Pin numbers
        _home(21), _extend(20), _retract(12), _half_extend(16) {
    _as.start();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pitch");

  PitchAction nav("pitch");
  ros::spin();
}
