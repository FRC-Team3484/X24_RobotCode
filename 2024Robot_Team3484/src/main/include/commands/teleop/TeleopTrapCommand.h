#ifndef TELEOP_TRAP_COMMAND_H
#define TELEOP_TRAP_COMMAND_H
#include "Constants.h"

#ifdef TRAP_ENABLED

#include "OI.h"
#include "subsystems/TrapSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class TeleopTrapCommand
    :public frc2::CommandHelper<frc2::Command, TeleopTrapCommand> {

    public:
        explicit TeleopTrapCommand(
            TrapSubsystem* trap_subsystem,
            Operator_Interface* oi
        );

        void Initialize() override;
        void Execute() override;
        void End(bool interupted) override;
        bool IsFinished() override;

    private:
        TrapSubsystem* _trap_subsystem;
        Operator_Interface* _oi;

    };

#endif
#endif