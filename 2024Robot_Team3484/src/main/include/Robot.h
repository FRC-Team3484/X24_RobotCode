// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef ROBOT_H
#define ROBOT_H

#include "OI.h"
#include "Constants.h"


// Swerve Stuff
#include "subsystems/DrivetrainSubsystem.h"
#include "commands/SwerveTeleop/TeleopDriveCommand.h"
#include "commands/SwerveTeleop/DynamicBrakeCommand.h"
#include "commands/SwerveTeleop/StraightenWheelsCommand.h"
#include "subsystems/AutonGenerator.h"


#include <string>
#include <optional>
#include <fmt/core.h>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>

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
  //void SimulationInit() override;
  //void SimulationPeriodic() override;

 private:
  enum State {drive, brake, straighten};
  State _robot_state = drive;

  OI _oi{};
  
  DrivetrainSubsystem _drivetrain{};

  TeleopDriveCommand _drive_command{&_drivetrain, &_oi};
  DynamicBrakeCommand _brake_command{&_drivetrain};
  StraightenWheelsCommand _straighten_command{&_drivetrain};

  AutonGenerator _auton_generator{&_drivetrain};

  std::optional<frc2::CommandPtr> _auton_command;
};

#endif