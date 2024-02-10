// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"


// Swerve Stuff
#include "subsystems/DrivetrainSubsystem.h"
#include "commands/teleop/TeleopAimCommand.h"
#include "commands/teleop/TeleopDriveCommand.h"
#include "subsystems/Vision.h"
// #include "commands/Teleop/StraightenWheelsCommand.h"
#include "subsystems/AutonGenerator.h"

// Other Subsystems
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LauncherSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

// Teleop Commands
#include "commands/teleop/TeleopClimberCommand.h"
#include "commands/teleop/TeleopIntakeCommand.h"
#include "commands/teleop/TeleopLauncherCommand.h"

#include <string>
#include <optional>
#include <fmt/core.h>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
class Robot : public frc::TimedRobot {
    public:
        void RobotInit() override;
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TestPeriodic() override;
        //void SimulationInit() override;w
        //void SimulationPeriodic() override;

    private:
        enum State {drive, shoot};
        State _robot_state = drive;

        Driver_Interface _oi_driver{};
        Operator_Interface _oi_operator{};

        Vision _vision{VisionConstants::CAMERA_ANGLE, VisionConstants::CAMERA_HEIGHT, VisionConstants::TARGET_HEIGHT};

        DrivetrainSubsystem _drivetrain{SwerveConstants::DrivetrainConstants::SWERVE_CONFIGS_ARRAY};

        // TeleopDriveCommand _drive_command{&_drivetrain, &_oi};
        // DynamicBrakeCommand _brake_command{&_drivetrain};

        // StraightenWheelsCommand _straighten_command{&_drivetrain};

        TeleopAimCommand _aim_command{&_drivetrain, &_oi_driver, &_oi_operator, &_vision};
        TeleopDriveCommand _drive_command{&_drivetrain, &_oi_driver};
        AutonGenerator _auton_generator{&_drivetrain};

        // Uncomment these when ready to test subsystems
        IntakeSubsystem _intake{IntakeConstants::PIVOT_MOTOR_CAN_ID, IntakeConstants::DRIVE_MOTOR_CAN_ID, IntakeConstants::PIECE_SENSOR_DI_CH, IntakeConstants::ARM_SENSOR_DI_CH, IntakeConstants::PID_CONSTANTS, IntakeConstants::PID_OUTPUTRANGE_MAX, IntakeConstants::PID_OUTPUTRANGE_MIN};
        LauncherSubsystem _launcher{LauncherConstants::LEFT_MOTOR_CAN_ID, LauncherConstants::RIGHT_MOTOR_CAN_ID, LauncherConstants::PID_CONSTANTS, LauncherConstants::RPM_WINDOW_RANGE};
        ClimberSubsystem _climber{ClimberConstants::LEFT_MOTOR_CAN_ID, ClimberConstants::RIGHT_MOTOR_CAN_ID, ClimberConstants::LEFT_SENSOR_DI_CH, ClimberConstants::RIGHT_SENSOR_DI_CH};
        
        // Uncomment these when ready to test the teleop commands
        TeleopClimberCommand _teleop_climber_command{&_climber, &_oi_operator};
        TeleopIntakeCommand _teleop_intake_command{&_intake, &_launcher, &_oi_operator};
        TeleopLauncherCommand _teleop_launcher_command{&_launcher, &_intake, &_vision, &_oi_operator};

        frc::DigitalInput _troubleshoot{0};

        std::optional<frc2::CommandPtr> _auton_command;
};

#endif