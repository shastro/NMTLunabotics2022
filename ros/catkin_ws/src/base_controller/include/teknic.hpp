// Teknic-specific motor controller implementation details.

// Copyright (c) 2021, 2022 NMT Lunabotics. All right reserved.

#ifndef H_TEKNIC
#define H_TEKNIC

#include <string>

#include "main.hpp"
#include "../src/teknic/motor_utils.hpp" // FIXME

class TeknicMotor : public MotorController {
public:
  // Initialize a new TeknicMotor. TODO: contructor details.
  TeknicMotor(SimpleNode &node);

  // Motor controller functions.
  void setVelocity(double vel);
  void setPosition(double pos);

private:
  // The SimpleNode backing this motor.
  SimpleNode &_node;
};

#endif // H_TEKNIC
