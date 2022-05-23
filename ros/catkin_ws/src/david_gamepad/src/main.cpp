// Bluetooth motor controller for NMT Lunabotics.
//
// Copyright (c) 2022 by NMT Lunabotics. All rights reserved.

#include "david_pitch/PitchCmd.h"
#include "joystick/8Bitdoh.hpp"
#include "joystick/joystick.hpp"
#include <david_motor/AugerCmd.h>
#include <david_motor/DepthCmd.h>
#include <david_motor/DumperCmd.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <unistd.h>
#include <vector>

using david_motor::AugerCmd;
using david_motor::DepthCmd;
using david_motor::DumperCmd;
using david_pitch::PitchCmd;
using geometry_msgs::Twist;
using ros::NodeHandle;
using ros::Publisher;

using namespace std;

static void joystick_sample_loop();
static Joystick joystick_connect();
static void handle_button(Pro2Button button, bool pressed);
static void handle_axis(Pro2Axis axis, short value);

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "david_gamepad");
  joystick_sample_loop();
}

// Connects to the joystick and the motors, and controls the motors
// according to the commands received.
static void joystick_sample_loop() {
  cout << "Connecting to joystick..." << endl;
  Joystick joystick = joystick_connect();
  cout << "Connected." << endl;

  cout << "Connecting to ROS..." << endl;
  NodeHandle node;
  cout << "Connected to ROS." << endl;

  Publisher augerPublisher = node.advertise<AugerCmd>("/cmd_auger", 16);
  Publisher dumperPublisher = node.advertise<DumperCmd>("/cmd_dumper", 16);
  Publisher velPublisher = node.advertise<Twist>("/cmd_vel", 16);
  Publisher depthPublisher = node.advertise<DepthCmd>("/cmd_depth", 16);
  Publisher pitchPublisher = node.advertise<PitchCmd>("/cmd_pitch", 16);
  cout << "Publishers set up." << endl;

  // Current position of right thumb joystick, because I'm too lazy to
  // come up with a better way of doing this.
  short rightJoyX = 0;
  short rightJoyY = 0;

  while (true) {
    JoystickEvent event = joystick.sample();

    if (event.isButton()) {
      Pro2Button button = static_cast<Pro2Button>(event.number);
      bool pressed = event.value == 1;

      switch (button) {
      case Pro2Button::B: {
        // Dump down
        DumperCmd cmd;
        if (pressed)
          cmd.vel = 100;
        else
          cmd.vel = 0;
        dumperPublisher.publish(cmd);
        break;
      }
      case Pro2Button::A: {
        // Unassigned
        break;
      }
      case Pro2Button::Y: {
        // Unassigned
        break;
      }
      case Pro2Button::X: {
        // Dump up
        DumperCmd cmd;
        if (pressed)
          cmd.vel = -100;
        else
          cmd.vel = 0;
        dumperPublisher.publish(cmd);
        break;
      }
      case Pro2Button::leftBumper: {
        // Pitch down
        PitchCmd cmd;
        if (pressed)
          cmd.spin = -1;
        else
          cmd.spin = 0;
        pitchPublisher.publish(cmd);
        break;
      }
      case Pro2Button::rightBumper: {
        // Pitch up
        PitchCmd cmd;
        if (pressed)
          cmd.spin = 1;
        else
          cmd.spin = 0;
        pitchPublisher.publish(cmd);
        break;
      }
      case Pro2Button::select: {
        // Unassigned
        break;
      }
      case Pro2Button::start: {
        if (pressed) {
          // Emergency stop
          AugerCmd acmd;
          acmd.spin = 0;
          augerPublisher.publish(acmd);

          DumperCmd dcmd;
          dcmd.vel = 0;
          dumperPublisher.publish(dcmd);

          Twist vcmd;
          vcmd.angular.x = 0;
          vcmd.angular.y = 0;
          vcmd.angular.z = 0;
          vcmd.linear.x = 0;
          vcmd.linear.y = 0;
          vcmd.linear.z = 0;
          velPublisher.publish(vcmd);

          PitchCmd pcmd;
          pcmd.spin = 0;
          pitchPublisher.publish(pcmd);
          return;
        }
      }
      case Pro2Button::thumbLeft: {
        // Unassigned
        break;
      }
      case Pro2Button::thumbRight:
        // Unassigned
        break;
      }
    } else if (event.isAxis()) {
      Pro2Axis axis = static_cast<Pro2Axis>(event.number);

      switch (axis) {
      case Pro2Axis::leftThumbX: {
        // Unassigned
        break;
      }
      case Pro2Axis::leftThumbY: {
        // Unassigned
        break;
      }
      case Pro2Axis::leftTrigger: {
        // Auger counterclockwise
        double weight = (event.value + 32768) / (double)65535;

        david_motor::AugerCmd cmd;
        cmd.spin = -weight * 4000;
        augerPublisher.publish(cmd);
        break;
      }
      case Pro2Axis::rightThumbX: {
        // Motion twist
        rightJoyX = event.value;

        double rot = -rightJoyX / 32768.0 * 500;
        double fwd = rightJoyY / 32768.0 * 500;

        Twist msg;
        msg.linear.x = fwd;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = rot;
        velPublisher.publish(msg);
        break;
      }
      case Pro2Axis::rightThumbY: {
        // Motion linear
        rightJoyY = event.value;

        double rot = -rightJoyX / 32768.0 * 500;
        double fwd = rightJoyY / 32768.0 * 500;

        Twist msg;
        msg.linear.x = fwd;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = rot;
        velPublisher.publish(msg);
        break;
      }
      case Pro2Axis::rightTrigger: {
        // Auger clockwise
        double weight = (event.value + 32768) / (double)65535;

        AugerCmd cmd;
        cmd.spin = weight * 4000;
        augerPublisher.publish(cmd);
        break;
      }
      case Pro2Axis::dpadX: {
        // Unassigned
        break;
      }
      case Pro2Axis::dpadY: {
        // Depth motor
        double weight = event.value / (double)65536;

        DepthCmd cmd;
        cmd.depth_vel = -weight * 100;
        depthPublisher.publish(cmd);
        break;
      }
      }
    }
  }
}

static Joystick joystick_connect() {
  while (true) {
    cout << "Waiting for joystick..." << endl;
    try {
      Joystick joystick("/dev/input/js0", true);
      return joystick;
    } catch (string err) {
      sleep(2);
    }
  }
}
