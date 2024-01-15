#include "commands/DriveSequenceCommand.h"

#include <units/length.h>
#include <units/angle.h>

#include <frc/geometry/Pose2d.h>

using namespace frc;

DriveSequenceCommand::DriveSequenceCommand(DrivetrainSubsystem* drivetrain) {
    AddCommands(
        GoToPoseCommand(drivetrain, Pose2d{6_ft, 0_ft, 90_deg}),
        GoToPoseCommand(drivetrain, Pose2d{6_ft, -3_ft, -90_deg}),
        GoToPoseCommand(drivetrain, Pose2d{0_ft, 0_ft, 0_deg})
    );
}