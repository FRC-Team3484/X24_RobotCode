// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef LAUNCHERCOMMANDH
#define LAUNCHERCOMMANDH

#include "subsystems/LauncherSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class LauncherCommand: public frc2::CommandHelper<frc2::Command, LauncherCommand>{
    public:
    explicit LauncherCommand(LauncherSubsystem* Launcher_subsytem);
    
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    
    private:
        LauncherSubsystem* _Launcher;
        bool _Launching; 

};

#endif