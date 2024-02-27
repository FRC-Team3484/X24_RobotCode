#ifndef AIMCOMMAND_H
#define AIMCOMMAND_H

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Constants.h"

#include "subsystems/DrivetrainSubsystem.h"
#include <frc/kinematics/SwerveModulePosition.h>
#include "subsystems/Vision.h"
#include "OI.h"

class TeleopAimCommand: public frc2::CommandHelper<frc2::Command, TeleopAimCommand>{

    public:
        explicit TeleopAimCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi_driver, Operator_Interface* oi_operator, Vision* vision);
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
        Driver_Interface* _oi_driver;
        Operator_Interface* _oi_operator;
        bool _aiming;
        frc::Timer _brake_timer;
        bool _encoder_saved;
        wpi::array<frc::SwerveModulePosition, 4> _current_positions = {
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad},
            frc::SwerveModulePosition{0_m, 0_rad}
        };
};

#endif