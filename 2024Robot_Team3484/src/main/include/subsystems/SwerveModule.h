#ifndef SWERVEMODULE_H
#define SWERVEMODULE_H

#include "Constants.h"


#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

class SwerveModule {
    public:
        SwerveModule(const int module_location);
        void SetDesiredState(frc::SwerveModuleState state, bool open_loop, bool optimize=true);
        frc::SwerveModuleState GetState();
        frc::SwerveModulePosition GetPosition();
        void ResetEncoder();
        void StopMotors();
        void SetCoastMode();
        void SetBrakeMode();

        
    private:
        WPI_TalonFX _drive_motor;
        WPI_TalonFX _steer_motor;
        WPI_CANCoder _steer_encoder;


        frc::PIDController _drive_pid_controller{SwerveConstants::DrivetrainConstants::DrivePIDConstants::P, SwerveConstants::DrivetrainConstants::DrivePIDConstants::I, SwerveConstants::DrivetrainConstants::DrivePIDConstants::D};
        frc::ProfiledPIDController<units::radians> _steer_pid_controller{SwerveConstants::DrivetrainConstants::SteerPIDConstants::P, SwerveConstants::DrivetrainConstants::SteerPIDConstants::I, SwerveConstants::DrivetrainConstants::SteerPIDConstants::D,
                {SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_SPEED, SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_ACCELRATION}};
        units::feet_per_second_t _GetWheelSpeed();
        units::inch_t _GetWheelPosition();
        units::degree_t _GetSteerAngle();

        frc::SimpleMotorFeedforward<units::meters> _drive_feed_forward{SwerveConstants::DrivetrainConstants::DriveFeedForwardConstants::S,SwerveConstants::DrivetrainConstants::DriveFeedForwardConstants::V,SwerveConstants::DrivetrainConstants::DriveFeedForwardConstants::A};
};

#endif
