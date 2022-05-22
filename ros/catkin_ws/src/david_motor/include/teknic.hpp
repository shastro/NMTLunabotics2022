// Teknic-specific motor controller implementation details.

// Copyright (c) 2021, 2022 NMT Lunabotics. All right reserved.

#ifndef H_TEKNIC
#define H_TEKNIC

#include <atomic>
#include <string>
#include <thread>

#include "../src/teknic/motor_utils.hpp" // FIXME
#include "main.hpp"

// Time resolution, in hertz, at which the motor manager loop runs.
// This is needed to prevent the motor from overloading its move
// queue.
#define MANAGER_RESOLUTION 2

// Maximum RMS value that the motor is willing to perform at. If the
// motor tries to spin above this value, we will artificially slow it
// down at this level to prevent drawing too much current.
#define MAX_RMS 0.75

class TeknicMotor : public MotorController {
public:
  // Initialize a new TeknicMotor.
  TeknicMotor(SimpleNode &node);

  // Motor controller functions.
  void setVelocity(double vel);
  void setPosition(double pos);

  // Get motor position (returns encoder count).
  double position();

  // Get motor velocity (returns RPM).
  double velocity();

  // Get measured torque (returns percentage of maximum by default).
  double torque();

  // Get measured rms_level (returns percentage of maximum).
  double rms();

private:
  // Thread that runs forever for each teknic motor; this manages the
  // motor's connection and monitors its RMS.
  void motor_manager();

  // The SimpleNode backing this motor.
  SimpleNode &_node;

  // The target velocity we're trying to reach. This is set by
  // `setVelocity()`, and is multiplied by an appropriate factor so as
  // not to exceed MAX_RMS and sent to the raw motor controller.
  std::atomic<double> _vel_target;

  // The current velocity of the motor.
  double _vel_current;

  // The thread running `motor_manager()`.
  std::thread _manager;
};

#endif // H_TEKNIC
