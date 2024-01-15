#ifndef GOTOPOSECOMMAND_H
#define GOTOPOSECOMMAND_H

#include "Constants.h"
#include "subsystems/DrivetrainSubsystem.h"

#include <units/length.h>
#include <units/angle.h>

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class GoToPoseCommand: public frc2::CommandHelper<frc2::CommandBase, GoToPoseCommand> {
    public:
        GoToPoseCommand(DrivetrainSubsystem* drivetrain, frc::Pose2d target);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override; 

    private:
        DrivetrainSubsystem* _drivetrain;

        frc::Pose2d _target_pose;
        frc::Pose2d _pose_delta;

        frc::TrapezoidProfile<units::meters>::Constraints _linear_constraints{
                DrivetrainConstants::MAX_LINEAR_SPEED, DrivetrainConstants::MAX_LINEAR_ACCELERATION};
        frc::TrapezoidProfile<units::radians>::Constraints _rotation_constraints{
                DrivetrainConstants::MAX_ROTATION_SPEED, DrivetrainConstants::MAX_ROTATION_ACCELERATION};
};


#endif