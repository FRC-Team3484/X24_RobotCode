#ifndef DEPLOYINTAKECOMMAND_H
#define DEPLOYINTAKECOMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/Intake.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class DeployIntakeCommand
    :public frc2::CommandHelper<frc2::CommandBase, DeployIntakeCommand> {

    public:
        explicit DeployIntakeCommand(IntakeSubsystem* intake_subsystem, OI *oi);
        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        void Stop();
        IntakeSubsystem* _intake_subsystem;
        OI* _oi;

};

#endif