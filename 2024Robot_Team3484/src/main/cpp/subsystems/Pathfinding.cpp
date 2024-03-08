#include "subsystems/Pathfinding.h"


// Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation
// frc::Pose2d targetPose = frc::Pose2d(10_m, 5_m, frc::Rotation2d(180_deg));

Pathfinding::Pathfinding() {}

frc2::CommandPtr Pathfinding::GoToPose(frc::Pose2d targetPose) {
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    frc2::CommandPtr pathfinding = pathplanner::AutoBuilder::pathfindToPose(
        targetPose,
        Pathfinding::constraints,
        0.0_mps, // Goal end velocity in meters/sec
        0.0_m // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );

    return pathfinding;
}