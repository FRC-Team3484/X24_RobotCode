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

        Vision* _limelight;
        Driver_Interface* _oi_driver;
        Operator_Interface* _oi_operator;
        bool _aiming;
};

#endif