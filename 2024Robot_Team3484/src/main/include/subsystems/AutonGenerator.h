#ifndef AUTON_GENERATOR_H
#define AUTON_GENERATOR_H

#include "subsystems/DrivetrainSubsystem.h"

//#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>

class AutonGenerator {
    public:
        AutonGenerator(DrivetrainSubsystem* drivetrain);
        frc2::CommandPtr GetAutonomousCommand();
        
    private:
        
        DrivetrainSubsystem* _drivetrain;
};

#endif