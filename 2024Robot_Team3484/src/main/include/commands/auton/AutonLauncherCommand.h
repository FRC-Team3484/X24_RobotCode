#ifndef AUTON_LAUNCHER_COMMAND_H
#define AUTON_LAUNCHER_COMMAND_H

#include "subsystems/LauncherSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/Vision.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

class AutonLauncherCommand: public frc2::CommandHelper<frc2::Command, AutonLauncherCommand>{
    public:
    explicit AutonLauncherCommand(LauncherSubsystem* launcher_subsystem, IntakeSubsystem* intake_subsystem, Vision* vision);
    
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    
    private:
        LauncherSubsystem* _launcher;
        IntakeSubsystem* _intake;
        Vision* _limelight;
        int _launching; 
        bool _loaded;
        frc::Timer _timer;
};

#endif
