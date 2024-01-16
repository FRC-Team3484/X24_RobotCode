#ifndef DYNAMICBRAKECOMMAND_H
#define DYNAMICBRAKECOMMAND_H

#include "subsystems/DrivetrainSubsystem.h"

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/kinematics/SwerveModulePosition.h>

class DynamicBrakeCommand: public frc2::CommandHelper<frc2::Command, DynamicBrakeCommand> {
    public:
        explicit DynamicBrakeCommand(DrivetrainSubsystem* drivetrain);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override; 

    private:
        DrivetrainSubsystem* _drivetrain;
        wpi::array<frc::SwerveModulePosition, 4> _initial_positions = {
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad}
        };
};

#endif