// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef LAUNCHERCOMMANDH
#define LAUNCHERCOMMANDH

#include "subsystems/LauncherSubsystem.h"
#include "teleop/TeleopIntakeCommand.h"
#include "subsystems/Vision.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>


class LauncherCommand: public frc2::CommandHelper<frc2::Command, LauncherCommand>{
    public:
    explicit LauncherCommand(LauncherSubsystem* launcher_Subsystem, IntakeSubsystem* intake_subsystem, Vision* vision, Operator_Interface* OI );
    
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    
    private:
        LauncherSubsystem* _Launcher;
        IntakeSubsystem* _intake;
        Vision* _limelight;
        Operator_Interface* _oi;
        bool _Launching; 
        bool _Loaded;

};

#endif