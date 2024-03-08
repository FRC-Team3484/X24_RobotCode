#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "Constants.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include "frc/geometry/Pose2d.h"

class Pathfinding{

    public:
        Pathfinding();
        frc2::CommandPtr GoToPose(frc::Pose2d targetPose);

    private:
        // Create the constraints to use while pathfinding
        pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
            SwerveConstants::AutonDriveConstants::MAX_LINEAR_SPEED, 
            SwerveConstants::AutonDriveConstants::MAX_LINEAR_ACCELERATION,
            SwerveConstants::AutonDriveConstants::MAX_ROTATION_SPEED, 
            SwerveConstants::AutonDriveConstants::MAX_ROTATION_ACCELERATION
        );

};

#endif