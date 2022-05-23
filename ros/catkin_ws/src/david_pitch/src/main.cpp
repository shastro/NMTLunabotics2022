// Pitch message dispatcher.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

#include "gpio_lib/rpi_gpio.hpp"
#include <david_pitch/PitchCmd.h>
#include <ros/ros.h>

using ros::NodeHandle;

using namespace std;

class PitchHandler {
private:
  NodeHandle handle_;
  GPIOOut modeSelect0_;
  GPIOOut modeSelect1_;
  GPIOOut enablePin_;

  ros::Subscriber pitchSubscriber_;

  void handleMsg(const david_pitch::PitchCmdConstPtr &msg) {
    switch (msg->spin) {
    case -1:;
      // Home
      cout << "Home" << endl;
      modeSelect0_ = false;
      modeSelect1_ = false;
      enablePin_ = true;
      break;

    case 0:;
      // Still
      cout << "Still" << endl;
      modeSelect0_ = false;
      modeSelect1_ = false;
      enablePin_ = false;
      break;

    case 1:;
      // Retract
      cout << "Retract" << endl;
      modeSelect0_ = true;
      modeSelect1_ = true;
      enablePin_ = true;
      break;

    default:;
      // ignore
    }
  }

public:
  PitchHandler()
      : handle_(), modeSelect0_(21), modeSelect1_(20), enablePin_(12),
        pitchSubscriber_(handle_.subscribe("/cmd_pitch", 1,
                                           &PitchHandler::handleMsg, this)) {
    cout << "Pitch handler set up" << endl;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pitch");

  PitchHandler handler;
  ros::spin();
}
