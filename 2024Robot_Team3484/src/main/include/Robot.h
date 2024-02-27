// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"


// Swerve Stuff
#include "frc2/command/Commands.h"
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
#include "subsystems/TrapSubsystem.h"

// Teleop Commands
#include "commands/teleop/TeleopClimberCommand.h"
#include "commands/teleop/TeleopIntakeCommand.h"
#include "commands/teleop/TeleopLauncherCommand.h"
#include "commands/teleop/TeleopTrapCommand.h"

#include <string>
#include <optional>
#include <fmt/core.h>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SendableChooser.h>
// Command Groups
#include <frc2/command/SequentialCommandGroup.h>
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
        void TestInit() override;
        void TestPeriodic() override;

    private:
        enum State {drive, shoot};
        State _robot_state = drive;

        // Interface OI
        Driver_Interface _oi_driver{};
        Operator_Interface _oi_operator{};

        //Subsystems
        #if defined (INTAKE_ENABLED) || defined (LAUNCHER_ENABLED)
        IntakeSubsystem _intake{IntakeConstants::PIVOT_MOTOR_CAN_ID, IntakeConstants::DRIVE_MOTOR_CAN_ID, IntakeConstants::PIECE_SENSOR_DI_CH, IntakeConstants::ARM_SENSOR_DI_CH, IntakeConstants::PIVOT_PID_CONSTANTS, IntakeConstants::PID_OUTPUTRANGE_MAX, IntakeConstants::PID_OUTPUTRANGE_MIN};
        #endif
        #if defined (TRAP_ENABLED)
        TrapSubsystem _trap{TrapConstants::EXTENSION_MOTOR_CAN_ID, TrapConstants::GP_CONTROL_CAN_ID, TrapConstants::PID_CONSTANTS, TrapConstants::PID_MAX, TrapConstants::PID_MIN};
        #endif
        #if defined (INTAKE_ENABLED) || defined (LAUNCHER_ENABLED)
        LauncherSubsystem _launcher{LauncherConstants::LEFT_MOTOR_CAN_ID, LauncherConstants::RIGHT_MOTOR_CAN_ID, LauncherConstants::LAUNCH_SENSOR_DI_CH,LauncherConstants::LEFT_PID_CONSTANTS, LauncherConstants::RIGHT_PID_CONSTANTS, LauncherConstants::RPM_WINDOW_RANGE};
        #endif
        #if defined (CLIMBER_ENABLED)
        ClimberSubsystem _climber{ClimberConstants::LEFT_MOTOR_CAN_ID, ClimberConstants::RIGHT_MOTOR_CAN_ID, ClimberConstants::LEFT_SENSOR_DI_CH, ClimberConstants::RIGHT_SENSOR_DI_CH};
        #endif

        DrivetrainSubsystem _drivetrain{SwerveConstants::DrivetrainConstants::SWERVE_CONFIGS_ARRAY};
        // Subsystem Adjacent
        Vision _vision{VisionConstants::CAMERA_ANGLE, VisionConstants::CAMERA_HEIGHT, VisionConstants::SPEAKER_TARGET_HEIGHT};
    

        AutonGenerator _auton_generator{&_drivetrain};

        // Command Groups
        frc2::CommandPtr _drive_state_commands = frc2::cmd::Parallel(
            #ifdef DRIVE_ENABLED
            TeleopDriveCommand{&_drivetrain, &_oi_driver}.ToPtr(),
            #endif
            #ifdef CLIMBER_ENABLED
            TeleopClimberCommand{&_climber, &_oi_operator}.ToPtr(),
            #endif
            #ifdef INTAKE_ENABLED
            TeleopIntakeCommand{&_intake, &_launcher, &_oi_operator, &_oi_driver}.ToPtr(),
            #endif
            #ifdef TRAP_ENABLED
            TeleopTrapCommand{&_trap, &_oi_operator}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        frc2::CommandPtr _launch_state_commands = frc2::cmd::Parallel(
            #ifdef AIM_ENABLED
            TeleopAimCommand{&_drivetrain, &_oi_driver, &_oi_operator, &_vision}.ToPtr(),
            #endif
            #ifdef LAUNCHER_ENABLED
            TeleopLauncherCommand{&_launcher, &_intake, &_vision, &_oi_operator}.ToPtr(),
            #endif
            frc2::cmd::None()
        );

        // Variables

        std::optional<frc2::CommandPtr> _auton_command;

        // Pipeline Stuff
        frc::SendableChooser<int> _pipeline_chooser;
        std::map<std::string, int> _pipeline_map;
        int _current_pipeline;
};

#endif
