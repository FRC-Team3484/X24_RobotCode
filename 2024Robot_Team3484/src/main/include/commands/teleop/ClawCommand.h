#ifndef TELEOPINTAKECOMMAND_H
#define TELEOPINTAKECOMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/ClawSubsystem.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ClawCommand
    :public frc2::CommandHelper<frc2::CommandBase, ClawCommand> {

    public:
        explicit ClawCommand(ClawSubsystem* intake_subsystem, OI *oi);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        ClawSubsystem* _claw_subsystem;
        OI* _oi;

};

#endif