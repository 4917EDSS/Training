// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSub.h"

DrivetrainSub::DrivetrainSub() = default;

// This method will be called once per scheduler run
void DrivetrainSub::Periodic() {}

void DrivetrainSub::drive(double leftPower, double rightPower) {
    m_leftMotor.Set(leftPower);
    m_rightMotor.Set(-rightPower);
}