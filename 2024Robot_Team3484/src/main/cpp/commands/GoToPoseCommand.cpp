#include "commands/GoToPoseCommand.h"

#include <units/time.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

using namespace frc;
using namespace units;
using namespace SwerveConstants::DrivetrainConstants;

GoToPoseCommand::GoToPoseCommand(DrivetrainSubsystem* drivetrain, Pose2d target)
    : _drivetrain{drivetrain}, _target_pose{target} {
    AddRequirements(_drivetrain);
    }

void GoToPoseCommand::Initialize() {}

void GoToPoseCommand::Execute() {
    //Get the linear and rotational distance to the target
    _pose_delta = _target_pose.RelativeTo(_drivetrain->GetPose());

    //Get the robot speed
    const ChassisSpeeds chassis_speeds = _drivetrain->GetChassisSpeeds();
    const meters_per_second_t linear_speed = units::math::sqrt(chassis_speeds.vx * chassis_speeds.vx + chassis_speeds.vy * chassis_speeds.vy);

    //Calculate the xy distance to the target
    const Translation2d linear_delta = _pose_delta.Translation();
    //Create trapezoid state for current direction (assumes velocity direction is towards target)
    const TrapezoidProfile<meters>::State current_linear_state{0_m, linear_speed};
    //Create trapezoid state for target position
    const TrapezoidProfile<meters>::State target_linear_state{linear_delta.Norm(), 0_mps};
    //Generate a trapezoid based on current velocity and target distance
    frc::TrapezoidProfile<meters> linear_profile = TrapezoidProfile<meters>{_linear_constraints, target_linear_state, current_linear_state};
    //Use the trapezoid to calculate the next velocity
    const meters_per_second_t linear_velocity = linear_profile.Calculate(20_ms).velocity;
    //Split the velocity into x and y components
    const meters_per_second_t x_velocity = linear_velocity * linear_delta.Angle().Cos();
    const meters_per_second_t y_velocity = linear_velocity * linear_delta.Angle().Sin();
    
    //Repeat above steps for angle
    const Rotation2d rotation_delta = _pose_delta.Rotation();
    const TrapezoidProfile<radians>::State current_rotation_state{0_rad, chassis_speeds.omega};
    const TrapezoidProfile<radians>::State target_rotation_state{rotation_delta.Radians(), 0_rad_per_s};
    frc::TrapezoidProfile<units::radians> rotation_profile = TrapezoidProfile<radians>{_rotation_constraints, target_rotation_state, current_rotation_state};
    const radians_per_second_t rotation_velocity = rotation_profile.Calculate(20_ms).velocity;

    //Apply the calculated velocities
    _drivetrain->Drive(x_velocity, y_velocity, rotation_velocity, false);
}

void GoToPoseCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
}

bool GoToPoseCommand::IsFinished() {
    return _pose_delta.Translation().Norm() <= POSITION_TOLERANCE && units::math::abs(_pose_delta.Rotation().Radians()) <= ANGLE_TOLERANCE;
}