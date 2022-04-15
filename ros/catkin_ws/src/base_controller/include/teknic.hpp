// Teknic-specific motor controller implementation details.

// Copyright (c) 2021, 2022 NMT Lunabotics. All right reserved.

#ifndef H_TEKNIC
#define H_TEKNIC

#include <string>

#include "main.hpp"

class TeknicMotor : public MotorController {
public:
  // Initialize a new TeknicMotor. TODO: contructor details.
  TeknicMotor();

  // Motor controller functions.
  void setVelocity(double vel);
  void setPosition(double pos);

private:
  // TODO: Implementation details.
};

#endif // H_TEKNIC
