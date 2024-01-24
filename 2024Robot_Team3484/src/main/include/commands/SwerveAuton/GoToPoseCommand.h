#ifndef GOTOPOSECOMMAND_H
#define GOTOPOSECOMMAND_H

#include "Constants.h"
#include "subsystems/DrivetrainSubsystem.h"

#include <units/length.h>
#include <units/angle.h>

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class GoToPoseCommand: public frc2::CommandHelper<frc2::Command, GoToPoseCommand> {
    public:
        GoToPoseCommand(DrivetrainSubsystem* drivetrain, frc::Pose2d target);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override; 

    private:
        DrivetrainSubsystem* _drivetrain;

        frc::ChassisSpeeds chassis_speeds;
        units::meters_per_second_t linear_speed;
        frc::Translation2d linear_delta;
        frc::TrapezoidProfile<units::meters>::State current_linear_state;
        frc::TrapezoidProfile<units::meters>::State target_linear_state;
        units::meters_per_second_t linear_velocity;
        units::meters_per_second_t x_velocity;
        units::meters_per_second_t y_velocity;

        frc::Rotation2d rotation_delta;

        units::radians_per_second_t rotation_velocity;


        frc::Pose2d _target_pose;
        frc::Pose2d _pose_delta;

        /*frc::TrapezoidProfile<units::meters> _linear_profile{
            frc::TrapezoidProfile<units::meters>::Constraints{
                SwerveConstants::AutonDriveConstants::MAX_LINEAR_SPEED, 
                SwerveConstants::AutonDriveConstants::MAX_LINEAR_ACCELERATION
            }
        };*/
        frc::TrapezoidProfile<units::meters> _linear_profile{{
                SwerveConstants::AutonDriveConstants::MAX_LINEAR_SPEED, 
                SwerveConstants::AutonDriveConstants::MAX_LINEAR_ACCELERATION
        }};

        frc::TrapezoidProfile<units::radians> _rotation_profile{{
                SwerveConstants::AutonDriveConstants::MAX_ROTATION_SPEED, 
                SwerveConstants::AutonDriveConstants::MAX_ROTATION_ACCELERATION
        }};
};


#endif