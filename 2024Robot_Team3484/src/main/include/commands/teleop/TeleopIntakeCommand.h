#ifndef TELEOPINTAKECOMMAND_H
#define TELEOPINTAKECOMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/IntakeSubsystem.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class TeleopIntakeCommand
    :public frc2::CommandHelper<frc2::CommandBase, TeleopIntakeCommand> {

    public:
        explicit TeleopIntakeCommand(IntakeSubsystem* intake_subsystem, OI *oi);

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        IntakeSubsystem* _intake_subsystem;
        OI* _oi;

};

#endif