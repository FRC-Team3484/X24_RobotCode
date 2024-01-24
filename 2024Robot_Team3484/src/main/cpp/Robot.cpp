// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>


using namespace SwerveConstants::AutonNames;

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

// void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    _auton_command = _auton_generator.GetAutonomousCommand();
  
  if (_auton_command) {
    _auton_command->Schedule();
  }

}

void Robot::AutonomousPeriodic() {}

// void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (_auton_command) {
    _auton_command->Cancel();
  }
  _robot_state = drive;
  _drive_command.Schedule();
}

void Robot::TeleopPeriodic() {
    switch (_robot_state) {
    case drive:
      if (_oi.GetBrake()) {
        _drive_command.Cancel();
        _brake_command.Schedule();
        _robot_state = brake;
      } else if (_oi.GetStraightenWheels()) {
        _drive_command.Cancel();
        _straighten_command.Schedule();
        _robot_state = straighten;
      }
      break;
    case brake:
      if (!_oi.GetBrake()) {
        _brake_command.Cancel();
        _drive_command.Schedule();
        _robot_state = drive;
      }
      break;
    case straighten:
      if (!_oi.GetStraightenWheels()) {
        _straighten_command.Cancel();
        _drive_command.Schedule();
        _robot_state = drive;
      }
      break;
    default:
      _robot_state = drive;
  }

}

// void Robot::TeleopExit() {}

// void Robot::TestInit() {
//   frc2::CommandScheduler::GetInstance().CancelAll();
// }

void Robot::TestPeriodic() {}

// void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
