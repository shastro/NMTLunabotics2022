// Navigational motor abstractions.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <memory>

#include "main.hpp"

NavMotor::NavMotor(MotorController *m, std::string name, double position,
                   double multiplier)
    : motor_(m), name_(name), position_(position), multiplier_(multiplier) {}

void NavMotor::nav(double linear, double angular) {
  motor_->setVelocity(multiplier_ * (linear + angular * -position_));
}

std::string NavMotor::name() { return name_; }

telemetry_msg NavMotor::telem() {
  telemetry_msg retval = {
      .position = motor_->position(),
      .velocity = motor_->velocity(),
      .torque = motor_->torque(),
      .rms = motor_->rms(),
  };
  return retval;
}
