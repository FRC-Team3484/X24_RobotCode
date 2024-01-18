#ifndef TELEOPDRIVECOMMAND_H
#define TELEOPDRIVECOMMAND_H

#include "OI.h"
#include "subsystems/DrivetrainSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class TeleopDriveCommand: public frc2::CommandHelper<frc2::Command, TeleopDriveCommand> {
    public:
        explicit TeleopDriveCommand(DrivetrainSubsystem* drivetrain, OI* oi);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override; 

    private:
        DrivetrainSubsystem* _drivetrain;
        OI* _oi;
};

#endif