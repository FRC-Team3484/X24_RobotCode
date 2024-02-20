// Copyright (c) FIRST and other WPILib contributors. Open Source Software; you can modify and/or share it under the terms of the WPILib BSD license file in the root directory of this project.

// This is the testing branch

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

using namespace SwerveConstants::AutonNames;
void Robot::RobotInit() {
    frc::SmartDashboard::PutBoolean("testing",true);
    // Choosing Pipelines
    _pipeline_map.emplace("Pipeline 0", 0);
    _pipeline_map.emplace("Pipeline 1", 1);
    _pipeline_chooser.SetDefaultOption("Pipeline 0", _pipeline_map.at("Pipeline 0"));
    _pipeline_chooser.AddOption("Pipeline 1", _pipeline_map.at("Pipeline 1"));
frc::SmartDashboard::PutData("Limelight Pipeline", &_pipeline_chooser);


}

void Robot::RobotPeriodic() {
    _vision.SetPipeline(_pipeline_chooser.GetSelected());


    

    // if 1; not on the switch; inverted
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

// void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    //_auton_command = _auton_generator.GetAutonomousCommand();

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
    _drive_state_commands.Schedule();
    _robot_state = drive;
}

// There are two states that can be done; drive and shoot
// drive: include logic for x-break using buttons
// shoot: break and visions
void Robot::TeleopPeriodic() {
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
void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}

#endif
