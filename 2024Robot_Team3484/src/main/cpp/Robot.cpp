// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is the testing branch

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

using namespace SwerveConstants::AutonNames;
void Robot::RobotInit() {
    frc::SmartDashboard::GetBoolean("testing",true);
    // _drive_commands = _GetDriveCommands();
    // _launch_commands = _GetLaunchCommands();

}

void Robot::RobotPeriodic() {
    frc::SmartDashboard::PutBoolean("Digital Input: 0",_troubleshoot.Get());
    // if 1; not on the switch; inverted
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
    _StartDriveCommands();
}

// There are two states that can be done; drive and shoot
// drive: include logic for x-break using buttons
// shoot: break and visions
void Robot::TeleopPeriodic() {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {}

    else {
        switch (_robot_state) {
        case drive:
            if (_oi_operator.Launch()) {
                _drive_state_commands.Cancel();
                _launch_state_commands.Schedule();

                _robot_state = shoot;
            }

            break;

        case shoot:
            if (!_oi_operator.Launch()) {
                _launch_state_commands.Cancel();
                _drive_state_commands.Schedule();
                _robot_state = drive;
            }

            break;
            default:
            _robot_state = drive;
        }

    }
}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}

#endif
