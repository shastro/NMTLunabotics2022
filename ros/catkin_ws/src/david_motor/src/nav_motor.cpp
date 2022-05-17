// Navigational motor abstractions.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <memory>

#include "main.hpp"

NavMotor::NavMotor(MotorController *m, double position, double multiplier)
    : motor_(m), position_(position), multiplier_(multiplier) {}

void NavMotor::nav(double linear, double angular) {
  motor_->setVelocity(multiplier_ * (linear + angular * -position_));
}
