#ifndef AUTONGENERATOR_H
#define AUTONGENERATOR_H

#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LauncherSubsystem.h"
#include "commands/auton/AutonAimCommand.h"
#include "subsystems/Vision.h"
//#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>

class AutonGenerator {
    public:
        AutonGenerator(DrivetrainSubsystem* drivetrain, AutonAimCommand* aimCommand);
        frc2::CommandPtr GetAutonomousCommand();
    private:
        //frc2::CommandPtr _BuildPathCommand(std::string path_name);

        

        //pathplanner::SwerveAutoBuilder* _auton_builder;
        //std::unordered_map<std::string, std::shared_ptr<frc2::Command>> _event_map;
        AutonAimCommand* _aim_command;
        // auto _aim_command_ptr = std::__make_shared<AutonAimCommand>(_aim_command);
        frc::SendableChooser<frc2::CommandPtr> _auton_chooser_initial;
        frc::SendableChooser<frc2::CommandPtr> _auton_chooser_piece_2;
        frc::SendableChooser<frc2::CommandPtr> _auton_chooser_peice_3;
        frc::SendableChooser<frc2::CommandPtr> _auton_chooser_piece_4;
};

#endif