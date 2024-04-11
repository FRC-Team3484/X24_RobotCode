#include "subsystems/AutonGenerator.h"
#include "Constants.h"

#include "commands/auton/AutonStopCommand.h"
#include "commands/auton/AutonSpoolCommand.h"

#include "commands/auton/AutonStopCommand.h"
#include "commands/auton/AutonSpoolCommand.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

using namespace frc;
using namespace pathplanner;
using namespace SwerveConstants::AutonNames;
using namespace SwerveConstants::AutonPoses;

AutonGenerator::AutonGenerator(DrivetrainSubsystem* drivetrain, LauncherSubsystem* launcher, IntakeSubsystem* intake, Vision* vision)
    : _drivetrain{drivetrain}, _launcher{launcher}, _intake{intake}, _vision{vision} {
    NamedCommands::registerCommand("LauncherCommand", std::move(frc2::cmd::Race(AutonLauncherCommand(launcher, intake, vision).ToPtr(), AutonAimCommand(drivetrain, vision).ToPtr())));
    NamedCommands::registerCommand("IntakeCommand", std::move(AutonIntakeCommand(intake).ToPtr()));
    NamedCommands::registerCommand("SpoolCommand", std::move(AutonSpoolCommand(launcher).ToPtr()));
    NamedCommands::registerCommand("TrapCommand", frc2::cmd::Wait(3_s));

    _auton_chooser_initial.AddOption("No", "No");
    _auton_chooser_initial.SetDefaultOption("Yes", "Yes");

    _auton_chooser_piece_2.SetDefaultOption("None","None");
    _auton_chooser_piece_3.SetDefaultOption("None","None");
    _auton_chooser_piece_4.SetDefaultOption("None","None");

    for (const std::string* ptr = std::begin(AUTON_NAMES); ptr != std::end(AUTON_NAMES); ptr++) {
        //_auton_map.emplace(*ptr, PathPlannerAuto(*ptr).ToPtr());
        _auton_chooser_piece_2.AddOption(*ptr, *ptr);
        _auton_chooser_piece_3.AddOption(*ptr, *ptr);
        _auton_chooser_piece_4.AddOption(*ptr, *ptr);
    }

    SmartDashboard::PutData("Launch Initial Piece", &_auton_chooser_initial);
    SmartDashboard::PutData("Path 1", &_auton_chooser_piece_2);
    SmartDashboard::PutData("Path 2", &_auton_chooser_piece_3);
    SmartDashboard::PutData("Path 3", &_auton_chooser_piece_4);
}

frc2::CommandPtr AutonGenerator::_GetCommand(std::string command_name) {
    if (command_name == "No" || command_name == "None") {
        return frc2::cmd::None();
    } else if (command_name == "Yes") {
        return frc2::cmd::Race(AutonLauncherCommand(_launcher, _intake, _vision).ToPtr(), AutonAimCommand(_drivetrain, _vision).ToPtr());
    } else {
        return PathPlannerAuto(command_name).ToPtr();
    }
}

frc2::CommandPtr AutonGenerator::GetAutonomousCommand() {
    if (_auton_chooser_piece_2.GetSelected() != "None") {
        _drivetrain->ResetOdometry(PathPlannerAuto::getStartingPoseFromAutoFile(_auton_chooser_piece_2.GetSelected()));
    }

    return frc2::cmd::Sequence(
        _GetCommand(_auton_chooser_initial.GetSelected()),
        frc2::cmd::Wait(.25_s),
        _GetCommand(_auton_chooser_piece_2.GetSelected()),
        frc2::cmd::Wait(.25_s),
        _GetCommand(_auton_chooser_piece_3.GetSelected()),
        frc2::cmd::Wait(.25_s),
        _GetCommand(_auton_chooser_piece_4.GetSelected()),
        AutonStopCommand(_drivetrain).ToPtr()
        //std::move(_auton_map.at(_auton_chooser_initial.GetSelected())),
        //std::move(_auton_map.at(_auton_chooser_piece_2.GetSelected())),
        //std::move(_auton_map.at(_auton_chooser_piece_3.GetSelected())),
        //std::move(_auton_map.at(_auton_chooser_piece_4.GetSelected()))
    );
}
