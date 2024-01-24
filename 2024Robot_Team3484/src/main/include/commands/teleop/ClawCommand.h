#ifndef TELEOPINTAKECOMMAND_H
#define TELEOPINTAKECOMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/ClawSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class ClawCommand
    :public frc2::CommandHelper<frc2::Command, ClawCommand> {

    public:
        explicit ClawCommand(ClawSubsystem* intake_subsystem, Driver_Interface *oi);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        ClawSubsystem* _claw_subsystem;
        Driver_Interface* _oi;

};

#endif