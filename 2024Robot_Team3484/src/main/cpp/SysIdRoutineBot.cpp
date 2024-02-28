// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SysIdRoutineBot.h"

#include <frc2/command/Commands.h>

SysIdRoutineBot::SysIdRoutineBot() {
  ConfigureBindings();
}

void SysIdRoutineBot::ConfigureBindings() {
  m_drive.SetDefaultCommand(m_drive.PseudoDriveCommand(
      [this] { return -m_driverController.GetLeftY(); },
      [this] { return -m_driverController.GetRightX(); }));

  // Using bumpers as a modifier and combining it with the buttons so that we
  // can have both sets of bindings at once
  (m_driverController.A())
      .WhileTrue(m_drive.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (m_driverController.B())
      .WhileTrue(m_drive.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  (m_driverController.X())
      .WhileTrue(m_drive.SysIdDynamic(frc2::sysid::Direction::kForward));
  (m_driverController.Y())
      .WhileTrue(m_drive.SysIdDynamic(frc2::sysid::Direction::kReverse));
}

frc2::CommandPtr SysIdRoutineBot::GetAutonomousCommand() {
  return m_drive.Run([] {});
}