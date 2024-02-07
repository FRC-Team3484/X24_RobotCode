#ifndef TELEOP_CLAW_COMMAND_H
#define TELEOP_CLAW_COMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/ClawSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class TeleopClawCommand
    :public frc2::CommandHelper<frc2::Command, TeleopClawCommand> {

    public:
        explicit TeleopClawCommand(ClawSubsystem* claw_subsystem, Operator_Interface* oi);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        ClawSubsystem* _claw_subsystem;
        Operator_Interface* _oi;

};

#endif