#ifndef AUTON_GENERATOR_H
#define AUTON_GENERATOR_H


// Subsystems
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LauncherSubsystem.h"
#include "subsystems/Vision.h"

//Auton Commands
#include "commands/auton/AutonAimCommand.h"
#include "commands/auton/AutonLauncherCommand.h"
#include "commands/auton/AutonIntakeCommand.h"
//#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>


class AutonGenerator {
    public:
        AutonGenerator(DrivetrainSubsystem* drivetrain, LauncherSubsystem* launcher, IntakeSubsystem* intake, Vision* vision);
        frc2::CommandPtr GetAutonomousCommand();
        
    private:

        std::unordered_map<std::string, frc2::CommandPtr> _auton_map;
        frc2::CommandPtr _GetCommand(std::string command_name);
        
        DrivetrainSubsystem* _drivetrain;
        IntakeSubsystem* _intake;
        LauncherSubsystem* _launcher;
        Vision* _vision;

        frc::SendableChooser<std::string> _auton_chooser_initial;
        frc::SendableChooser<std::string> _auton_chooser_piece_2;
        frc::SendableChooser<std::string> _auton_chooser_piece_3;
        frc::SendableChooser<std::string> _auton_chooser_piece_4;
};

#endif