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

//Autons
#include "commands/auton/AutonAimCommand.h"


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

  // Creation of Command for Path
  AutonAimCommand _aim_command_auton{&_drivetrain, &_vision};
  

  AutonGenerator _auton_generator{&_drivetrain, &_aim_command_auton};

  frc::DigitalInput _troubleshoot{0};

  std::optional<frc2::CommandPtr> _auton_command;
};

#endif