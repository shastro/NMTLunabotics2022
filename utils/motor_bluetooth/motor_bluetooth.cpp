// Bluetooth motor controller for NMT Lunabotics.
//
// Copyright (c) 2022 by NMT Lunabotics. All rights reserved.

#include "motor/motor_utils.hpp"
#include "joystick/joystick.hpp"
#include "joystick/8Bitdoh.hpp"
#include <unistd.h>
#include <vector>

using namespace std;
using namespace sFnd;

static void joystick_sample_loop();
static ButtonCommand button_control_scheme(Pro2Button button);
static AxisCommand axis_control_scheme(Pro2Axis axis);
static Joystick joystick_connect();

int main(int argc, char *argv[]) {
  try {
    joystick_sample_loop();
  } catch (string err) {
    cerr << "Motor controller error: " << err << endl;
    return EXIT_FAILURE;
  }
}

// Button command constructor.
ButtonCommand::ButtonCommand(vector<MotorID> _motors, int _velocity)
    : motors(_motors), velocity(_velocity) {}

// Axis conmmand constructor.
AxisCommand::AxisCommand(vector<MotorID> _motors, int _basis, int _velocity)
    : motors(_motors), basis(_basis), velocity(_velocity) {}

// Connects to the joystick and the motors, and controls the motors
// according to the commands received.
static void joystick_sample_loop() {
  cout << "Connecting to joystick..." << endl;
  Joystick joystick = joystick_connect();
  cout << "Connected." << endl;

  cout << "Connecting to ports..." << endl;
  vector<SimplePort> ports = SimplePort::getPorts();
  cout << "Connected." << endl;

  SimplePort &port = ports.at(0);

  while (true) {
    JoystickEvent event = joystick.sample();

    vector<MotorID> motors;
    int targetVelocity;

    if (event.isButton()) {
      Pro2Button button = static_cast<Pro2Button>(event.number);

      ButtonCommand cmd = button_control_scheme(button);
      motors = cmd.motors;
      targetVelocity = ((event.value == 1) ? 1 : 0) * cmd.velocity;
    } else if (event.isAxis()) {
      Pro2Axis axis = static_cast<Pro2Axis>(event.number);

      AxisCommand cmd = axis_control_scheme(axis);
      motors = cmd.motors;
      targetVelocity = (event.value - cmd.basis) * cmd.velocity * (1.0 / 65536);
    }

    for (MotorID &motor : motors) {
      try {
        port.getNodes().at(motor).setVel(targetVelocity);
      } catch (string err) {
        cout << "Motor " << motor << " error: " << err << endl;
      }
    }
  }
}

// Get the list of motors that need to move when a button is pressed
// or released.
static ButtonCommand button_control_scheme(Pro2Button button) {
  // X -> dump up
  // B -> dump down

  // Arduino commands:
  // leftBumper -> lower
  // rightBumper -> raise
  switch (button) {
  // case Pro2Button::X:
  //   return ButtonCommand({MotorIdent::DumpR, MotorIdent::DumpL}, 30);

  // case Pro2Button::B:
  //   return ButtonCommand({MotorIdent::DumpR, MotorIdent::DumpL}, -30);

  case Pro2Button::start:
    // Stop every motor.
    return ButtonCommand(
        {
            MotorIdent::Auger,
            // MotorIdent::DumpL,
            MotorIdent::DepthL,
            MotorIdent::LocomotionL,
            MotorIdent::LocomotionR,
            MotorIdent::DepthR,
            // MotorIdent::DumpR,
        },
        0);

  default:
    return ButtonCommand({}, 0);
  }
}

// Get the list of motors that need to move when a trigger (axis) is
// pressed or released.
static AxisCommand axis_control_scheme(Pro2Axis axis) {
  // dpadY -> depth
  // rightTrigger -> auger clockwise
  // leftTrigger -> auger counterclockwise
  // leftThumbY -> left locomotion
  // rightThumbY -> right locomotion
  switch (axis) {
  case Pro2Axis::dpadY:
    return AxisCommand({MotorIdent::DepthL, MotorIdent::DepthR}, 0, 30);

  case Pro2Axis::rightTrigger:
    return AxisCommand({MotorIdent::Auger}, -32767, 1000);

  case Pro2Axis::leftTrigger:
    return AxisCommand({MotorIdent::Auger}, -32767, -1000);

  case Pro2Axis::leftThumbY:
    return AxisCommand({MotorIdent::LocomotionL}, 0, -500);

  case Pro2Axis::rightThumbY:
    return AxisCommand({MotorIdent::LocomotionR}, 0, 500);

  default:
    return AxisCommand({}, 0, 0);
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
