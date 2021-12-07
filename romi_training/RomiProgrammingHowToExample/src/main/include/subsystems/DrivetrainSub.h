// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/BuiltInAccelerometer.h>
#include <frc/Encoder.h>
#include <frc/Spark.h>
//#include "sensors/RomiGyro.h"

#include <frc2/command/SubsystemBase.h>

class DrivetrainSub : public frc2::SubsystemBase {
 public:
  DrivetrainSub();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void drive(double leftPower, double rightPower);  // Power is -1.0 to 1.0

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Spark m_leftMotor{0};
  frc::Spark m_rightMotor{1};

  frc::Encoder m_leftEncoder{4, 5};
  frc::Encoder m_rightEncoder{6, 7};

  frc::BuiltInAccelerometer m_accelerometer;
  //RomiGyro m_gyro;

};
