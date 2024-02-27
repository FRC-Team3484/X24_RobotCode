#include "subsystems/AutonGenerator.h"
#include "Constants.h"

//#include <pathplanner/lib/PathPlannerTrajectory.h>
//#include <pathplanner/lib/PathPlanner.h>
//#include <pathplanner/lib/commands/FollowPathWithEvents.h>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
//using namespace pathplanner;
using namespace SwerveConstants::AutonNames;

AutonGenerator::AutonGenerator(DrivetrainSubsystem* drivetrain) 
    : _drivetrain{drivetrain} {
}

frc2::CommandPtr AutonGenerator::GetAutonomousCommand() {
  return frc2::cmd::None();
}