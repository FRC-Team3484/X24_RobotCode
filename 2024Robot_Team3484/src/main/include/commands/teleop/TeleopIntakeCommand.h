#ifndef TELEOP_INTAKE_COMMAND_H
#define TELEOP_INTAKE_COMMAND_H

#include "OI.h"
#include "Constants.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LauncherSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class TeleopIntakeCommand
    :public frc2::CommandHelper<frc2::Command, TeleopIntakeCommand> {

    public:
        explicit TeleopIntakeCommand(
            IntakeSubsystem* intake_subsystem,
            LauncherSubsystem* launcher_subsystem,
            ClimberSubsystem* climber_subsystem,
            Operator_Interface* operator_oi,
            Driver_Interface* driver_oi
        );

        void Initialize() override;
        void Execute() override;
        void End(bool inturrupted) override;
        bool IsFinished() override;

    private:
        IntakeSubsystem* _intake_subsystem;
        LauncherSubsystem* _launcher_subsystem;
        Operator_Interface* _operator_oi;
        Driver_Interface* _driver_oi;
        ClimberSubsystem* _climber_subsystem;
};

#endif