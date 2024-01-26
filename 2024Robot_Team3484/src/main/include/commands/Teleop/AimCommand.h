#ifndef AIMCOMMAND_H
#define AIMCOMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Constants.h"

#include "subsystems/DrivetrainSubsystem.h"
#include <frc/kinematics/SwerveModulePosition.h>
#include "subsystems/Vision.h"



class AimCommand: public frc2::CommandHelper<frc2::Command, AimCommand>{

    public:
        explicit AimCommand(DrivetrainSubsystem* drivetrain, Vision* vision);
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

        Vision* _limelight;
        bool _aiming;


};





#endif