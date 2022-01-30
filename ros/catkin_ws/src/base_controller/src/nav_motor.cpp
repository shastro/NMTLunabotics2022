// Navigational motor abstractions.

// Copyright (c) 2021, 2022 NMT Lunabotics. All rights reserved.

#include <memory>

#include "main.hpp"

NavMotor::NavMotor(MotorController *m, double position)
    : motor_(m), position_(position) {}

void NavMotor::nav(double linear, double angular) {
  motor_->setVelocity(linear + angular * -position_);
}
