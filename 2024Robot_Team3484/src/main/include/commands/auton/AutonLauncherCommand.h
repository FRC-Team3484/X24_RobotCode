// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef AUTONLAUNCHERCOMMANDH
#define AUTONLAUNCHERCOMMANDH

#include "subsystems/LauncherSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/Vision.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>


class AutonLauncherCommand: public frc2::CommandHelper<frc2::Command, AutonLauncherCommand>{
    public:
    explicit AutonLauncherCommand(LauncherSubsystem* Launcher_Subsystem, IntakeSubsystem* intake_subsystem, Vision* vision);
    
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    
    private:
        LauncherSubsystem* _launcher;
        IntakeSubsystem* _intake;
        Vision* _limelight;
        bool _launching; 
        bool _loaded;
        frc::Timer _timer;
        units::second_t _launch_time;
};

#endif