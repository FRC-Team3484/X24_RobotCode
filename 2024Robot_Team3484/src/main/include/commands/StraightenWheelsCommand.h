#ifndef STRAIGHTENWHEELSCOMMAND_H
#define STRAIGHTENWHEELSCOMMAND_H

#include "subsystems/DrivetrainSubsystem.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class StraightenWheelsCommand: public frc2::CommandHelper<frc2::CommandBase, StraightenWheelsCommand> {
    public:
        explicit StraightenWheelsCommand(DrivetrainSubsystem* drivetrain);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override; 

    private:
        DrivetrainSubsystem* _drivetrain;
};

#endif