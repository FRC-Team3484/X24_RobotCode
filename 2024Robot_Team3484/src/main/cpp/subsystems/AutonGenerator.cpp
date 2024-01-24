#include "subsystems/AutonGenerator.h"
#include "Constants.h"
#include "commands/SwerveAuton/GoToPoseCommand.h"
#include "commands/SwerveAuton/DriveSequenceCommand.h"

//#include <pathplanner/lib/PathPlannerTrajectory.h>
//#include <pathplanner/lib/PathPlanner.h>
//#include <pathplanner/lib/commands/FollowPathWithEvents.h>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
//using namespace pathplanner;
using namespace SwerveConstants::AutonNames;

AutonGenerator::AutonGenerator(DrivetrainSubsystem* drivetrain) 
    : _drivetrain{drivetrain} {

    _auton_chooser.SetDefaultOption(AUTON_NONE, AUTON_NONE);
    _auton_chooser.AddOption(AUTON_DISTANCE, AUTON_DISTANCE);
    _auton_chooser.AddOption(AUTON_ANGLE, AUTON_ANGLE);
    _auton_chooser.AddOption(AUTON_SEQUENCE, AUTON_SEQUENCE);
    frc::SmartDashboard::PutData("Autons", &_auton_chooser);

    //_event_map.emplace("test", std::make_shared<frc2::PrintCommand>("test"));

    /*_auton_builder = new SwerveAutoBuilder{
        [this](){return _drivetrain->GetPose();},
        [this](Pose2d pose){_drivetrain->ResetOdometry(pose);},
        _drivetrain->kinematics,
        PIDConstants(PathDrivePIDConstants::P, PathDrivePIDConstants::I, PathDrivePIDConstants::D),
        PIDConstants(PathRotationPIDConstants::P, PathRotationPIDConstants::I, PathRotationPIDConstants::D),
        [this](wpi::array<SwerveModuleState, 4> states){_drivetrain->SetModuleStates(states, false, true);},
        _event_map,
        {_drivetrain},
        true
    };*/
}

frc2::CommandPtr AutonGenerator::GetAutonomousCommand() {
  const std::string auton_selected = _auton_chooser.GetSelected();

  if (auton_selected == AUTON_NONE) {
    fmt::print("No Auton Selected");
  } else if (auton_selected == AUTON_DISTANCE) {
    return GoToPoseCommand{_drivetrain, Pose2d{5_ft, 0_ft, 0_deg}}.ToPtr();
  } else if (auton_selected == AUTON_ANGLE) {
    return GoToPoseCommand{_drivetrain, Pose2d{0_ft, 0_ft, 90_deg}}.ToPtr();
  } else if (auton_selected == AUTON_SEQUENCE) {
    return DriveSequenceCommand{_drivetrain}.ToPtr();
  }
  return frc2::cmd::None();
}

/*frc2::CommandPtr AutonGenerator::_BuildPathCommand(std::string path_name) {
    PathPlannerTrajectory path = PathPlanner::loadPath(path_name, PathConstraints(MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION));
    
    return _auton_builder->fullAuto(path);
}*/