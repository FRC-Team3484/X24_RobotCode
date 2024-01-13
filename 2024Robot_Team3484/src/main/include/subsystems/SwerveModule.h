#ifndef SWERVEMODULE_H
#define SWERVEMODULE_H

#include "Constants.h"

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <ctre/Phoenix.h>
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/smartdashboard/SmartDashboard.h>

class SwerveModule {
    public:
        SwerveModule(const int module_location);
        void SetDesiredState(const frc::SwerveModuleState& state, bool open_loop);
        frc::SwerveModuleState GetState();
        frc::SwerveModulePosition GetPosition();
        void ResetEncoder();
        
    private:
        WPI_TalonFX _drive_motor;
        WPI_TalonFX _steer_motor;
        WPI_CANCoder _steer_encoder;

        frc::PIDController _drive_pid_controller{DrivePIDConstants::P, DrivePIDConstants::I, DrivePIDConstants::D};
        frc::ProfiledPIDController<units::radians> _steer_pid_controller{SteerPIDConstants::P, SteerPIDConstants::I, SteerPIDConstants::D,
                {SteerPIDConstants::MAX_SPEED, SteerPIDConstants::MAX_ACCELRATION}};
        units::feet_per_second_t _GetWheelSpeed();
        units::inch_t _GetWheelPosition();
        units::degree_t _GetSteerAngle();

        frc::SimpleMotorFeedforward<units::meters> _dirve_feed_forward{DriveFeedForwardConstants::S,DriveFeedForwardConstants::V,DriveFeedForwardConstants::A};
};

#endif
