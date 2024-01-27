#ifndef AUTONINTAKECOMMAND_H
#define AUTONINTAKECOMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class AutonIntakeCommand
    :public frc2::CommandHelper<frc2::Command, AutonIntakeCommand> {

    public:
        explicit AutonIntakeCommand(IntakeSubsystem* intake_subsystem);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        IntakeSubsystem* _intake_subsystem;

};

#endif