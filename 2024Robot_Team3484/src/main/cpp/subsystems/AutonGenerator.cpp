#include "subsystems/AutonGenerator.h"
#include "Constants.h"
#include "commands/auton/GoToPoseCommand.h"
#include "commands/auton/DriveSequenceCommand.h"

//#include <pathplanner/lib/PathPlannerTrajectory.h>
// #include <pathplanner/lib/PathPlanner.h>
//#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

using namespace frc;
using namespace pathplanner;
using namespace SwerveConstants::AutonNames;

AutonGenerator::AutonGenerator(DrivetrainSubsystem* drivetrain, LauncherSubsystem* launcher, IntakeSubsystem* intake, Vision* vision)
     {
      NamedCommands::registerCommand("LauncherCommand", std::move(frc2::cmd::Race(AutonLauncherCommand(launcher, intake, vision).ToPtr(), AutonAimCommand(drivetrain, vision).ToPtr())));
      NamedCommands::registerCommand("IntakeCommand", std::move(AutonIntakeCommand(intake).ToPtr()));



    _auton_chooser_initial.AddOption("No", "No");
    _auton_chooser_initial.SetDefaultOption("Yes", "Yes");

    _auton_chooser_piece_2.SetDefaultOption("None","None");
    _auton_chooser_piece_3.SetDefaultOption("None","None");
    _auton_chooser_piece_4.SetDefaultOption("None","None");


    _auton_map.emplace("No", frc2::cmd::None());
    _auton_map.emplace("None", frc2::cmd::None());
    _auton_map.emplace("Yes", frc2::cmd::Race(AutonLauncherCommand(launcher, intake, vision).ToPtr(), AutonAimCommand(drivetrain, vision).ToPtr()));




    for (const std::string* ptr = std::begin(AUTON_NAMES); ptr != std::end(AUTON_NAMES); ptr++){
      _auton_map.emplace(*ptr, PathPlannerAuto(*ptr).ToPtr());
      _auton_chooser_piece_2.AddOption(*ptr, *ptr);
      _auton_chooser_piece_3.AddOption(*ptr, *ptr);
      _auton_chooser_piece_4.AddOption(*ptr, *ptr);
    }
}

frc2::CommandPtr AutonGenerator::GetAutonomousCommand() {
  return frc2::cmd::Sequence(
    std::move(_auton_map.at(_auton_chooser_initial.GetSelected())),
    std::move(_auton_map.at(_auton_chooser_piece_2.GetSelected())),
    std::move(_auton_map.at(_auton_chooser_piece_3.GetSelected())),
    std::move(_auton_map.at(_auton_chooser_piece_4.GetSelected()))
  );
}
