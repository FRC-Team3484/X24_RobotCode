#ifndef TELEOP_CLIMBER_COMMAND_H
#define TELEOP_CLIMBER_COMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/ClimberSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class TeleopClimberCommand
    :public frc2::CommandHelper<frc2::Command, TeleopClimberCommand> {

    public:
        explicit TeleopClimberCommand(ClimberSubsystem* Climber_subsystem, Operator_Interface* oi);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        ClimberSubsystem* _climber_subsystem;
        Operator_Interface* _oi;

        #ifdef EN_TESTING
        bool _is_open_loop;
        #endif

};

#endif