// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef SHOOTERCOMMAND_H
#define SHOOTERCOMMAND_H

#include "subsystems/ShooterSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class ShooterCommand: public frc2::CommandHelper<frc2::Command, ShooterCommand>{
    public:
    explicit ShooterCommand(ShooterSubsystem* shooter_subsytem);
    
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    
    private:
        ShooterSubsystem* _shooter;
        bool _shooting;

};

#endif