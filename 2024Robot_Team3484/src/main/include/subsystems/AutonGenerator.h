#ifndef AUTONGENERATOR_H
#define AUTONGENERATOR_H

#include "subsystems/DrivetrainSubsystem.h"

#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>

class AutonGenerator {
    public:
        AutonGenerator(DrivetrainSubsystem* drivetrain);
        frc2::CommandPtr GetAutonomousCommand();
    private:
        frc2::CommandPtr _BuildPathCommand(std::string path_name);
        
        DrivetrainSubsystem* _drivetrain;

        pathplanner::SwerveAutoBuilder* _auton_builder;
        std::unordered_map<std::string, std::shared_ptr<frc2::Command>> _event_map;

        frc::SendableChooser<std::string> _auton_chooser;
        std::optional<frc2::CommandPtr> _auton_command;
};

#endif