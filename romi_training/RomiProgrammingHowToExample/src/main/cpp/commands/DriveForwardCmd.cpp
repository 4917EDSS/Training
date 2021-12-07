// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForwardCmd.h"

DriveForwardCmd::DriveForwardCmd(DrivetrainSub* drivetrainSub) : m_drivetrainSubPtr(drivetrainSub) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrainSub});
}

// Called when the command is initially scheduled.
void DriveForwardCmd::Initialize() {
  m_drivetrainSubPtr->drive(0.5, 0.5);
}

// Called repeatedly when this Command is scheduled to run
void DriveForwardCmd::Execute() {}

// Called once the command ends or is interrupted.
void DriveForwardCmd::End(bool interrupted) {
  m_drivetrainSubPtr->drive(0.0, 0.0);
}

// Returns true when the command should end.
bool DriveForwardCmd::IsFinished() {
  return false;
}
