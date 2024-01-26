#ifndef DRIVECOMMAND_H
#define DRIVECOMMAND_H

#include "OI.h"
#include "subsystems/DrivetrainSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/kinematics/SwerveModulePosition.h>

class DriveCommand: public frc2::CommandHelper<frc2::Command, DriveCommand> {
    public:
        explicit DriveCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override; 

    private:
        DrivetrainSubsystem* _drivetrain;
        Driver_Interface* _oi;
        wpi::array<frc::SwerveModulePosition, 4> _initial_positions = {
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad}
        };
};


#endif