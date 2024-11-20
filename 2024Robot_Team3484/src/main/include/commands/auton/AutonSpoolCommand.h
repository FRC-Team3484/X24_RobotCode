#ifndef AUTONSPOOLCOMMAND_H
#define AUTONSPOOLCOMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/LauncherSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class AutonSpoolCommand
    :public frc2::CommandHelper<frc2::Command, AutonSpoolCommand> {

    public:
        explicit AutonSpoolCommand(LauncherSubsystem* launcher_subsystem);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        LauncherSubsystem* _launcher_subsystem;

};

#endif