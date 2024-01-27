#include "commands/auton/GoToPoseCommand.h"

#include <units/time.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace units;
using namespace SwerveConstants::AutonDriveConstants;

GoToPoseCommand::GoToPoseCommand(DrivetrainSubsystem* drivetrain, Pose2d target)
    : _drivetrain{drivetrain}, _target_pose{target} {
    AddRequirements(_drivetrain);
    }

void GoToPoseCommand::Initialize() {}

void GoToPoseCommand::Execute() {
    //Get the linear and rotational distance to the target
    _pose_delta = _target_pose.RelativeTo(_drivetrain->GetPose());
    SmartDashboard::PutNumber("X Delta", _pose_delta.X().value());
    SmartDashboard::PutNumber("Y Delta", _pose_delta.Y().value());
    SmartDashboard::PutNumber("A Delta", _pose_delta.Rotation().Degrees().value());
    SmartDashboard::PutNumber("Angle", _drivetrain->GetHeading().Degrees().value());

    //Get the robot speed
    chassis_speeds = _drivetrain->GetChassisSpeeds();
    SmartDashboard::PutNumber("Rotational Velocity", chassis_speeds.omega.value());
    linear_speed = units::math::sqrt(chassis_speeds.vx * chassis_speeds.vx + chassis_speeds.vy * chassis_speeds.vy);

    //Calculate the xy distance to the target
    linear_delta = _pose_delta.Translation();
    //Use the trapezoid to calculate the next velocity
    linear_velocity = _linear_profile.Calculate(20_ms, current_linear_state, target_linear_state).velocity;

    // Faulty
    // current_linear_state{0_m, linear_speed};
    // target_linear_state{linear_delta.Norm(), 0_mps};

    //Create trapezoid state for current direction (assumes velocity direction is towards target)
    frc::TrapezoidProfile<units::meters>::State current_linear_state{0_m, linear_speed};

    //Create trapezoid state for target position
    frc::TrapezoidProfile<units::meters>::State target_linear_state{linear_delta.Norm(), 0_mps};
    

    //Split the velocity into x and y components
    x_velocity = linear_velocity * linear_delta.Angle().Cos();
    y_velocity = linear_velocity * linear_delta.Angle().Sin();
    
    //Repeat above steps for angle
    Rotation2d rotation_delta = _pose_delta.Rotation();
    TrapezoidProfile<radians>::State current_rotation_state{0_rad, chassis_speeds.omega};
    TrapezoidProfile<radians>::State target_rotation_state{rotation_delta.Radians(), 0_rad_per_s};
    radians_per_second_t rotation_velocity = _rotation_profile.Calculate(20_ms, current_rotation_state, target_rotation_state).velocity;

    //Apply the calculated velocities
    _drivetrain->Drive(x_velocity, y_velocity, rotation_velocity, false);
}

void GoToPoseCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
}

bool GoToPoseCommand::IsFinished() {
    return _pose_delta.Translation().Norm() <= POSITION_TOLERANCE && units::math::abs(_pose_delta.Rotation().Radians()) <= ANGLE_TOLERANCE;
}