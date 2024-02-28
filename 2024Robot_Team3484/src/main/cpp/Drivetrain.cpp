// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <frc2/command/Commands.h>

Drive::Drive() {
 _drive_motor_BR.Follow(_drive_motor_FR);
 _drive_motor_BL.Follow(_drive_motor_FL);
//  _drive_motor_BL.SetInverted(false);
 _drive_motor_FL.SetInverted(false);
//  _drive_motor_BR.SetInverted(false);
 _drive_motor_FR.SetInverted(false);

 m_drive.SetSafetyEnabled(false);

}

frc2::CommandPtr Drive::PseudoDriveCommand(std::function<double()> fwd,
                                           std::function<double()> rot) {
  return frc2::cmd::Run([this, fwd, rot] { m_drive.ArcadeDrive(fwd(), rot()); },
                        {this})
      .WithName("Psuedo Testing Arcade Drive");
}

frc2::CommandPtr Drive::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Drive::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}
