#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include "Constants.h"

// #include <ctre/phoenix6/TalonFX.hpp>
// #include <ctre/phoenix6/CANcoder.hpp>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <wpi/deprecated.h>

WPI_IGNORE_DEPRECATED

#include "ctre/phoenix/sensors/WPI_CANCoder.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
// #include "ctre/Phoenix.h"

class SwerveModule {
    public:
        SwerveModule(SC::SC_SwerveConfigs corner);
        void SetDesiredState(frc::SwerveModuleState state, bool open_loop, bool optimize=true);
        frc::SwerveModuleState GetState();
        frc::SwerveModulePosition GetPosition();
        void StopMotors();
        void ResetEncoder();
        void SetCoastMode();
        void SetBrakeMode();

    private:
        // ctre::phoenix6::configs::TalonFXConfiguration _drive_motor_config{};

        // ctre::phoenix6::hardware::TalonFX _drive_motor;
        // ctre::phoenix6::hardware::TalonFX _steer_motor;
        // ctre::phoenix6::hardware::CANcoder _steer_encoder;
        // Check for proper path declaration

        ctre::phoenix::motorcontrol::can::WPI_TalonFX _drive_motor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX _steer_motor;
        ctre::phoenix::sensors::WPI_CANCoder _steer_encoder;


        // WPI_TalonFX _drive_motor;
        // WPI_TalonFX _steer_motor;
        // WPI_CANCoder _steer_encoder;

        SC::SC_SwerveCurrents _swerve_current_constants;

        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration _drive_currrent_limit{
            _swerve_current_constants.Current_Limit_Enable,
            _swerve_current_constants.Current_Limit_Drive,
            _swerve_current_constants.Drive_Current_Threshold,
            _swerve_current_constants.Drive_Current_Time
        };
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration _steer_current_limit{
            _swerve_current_constants.Current_Limit_Enable,
            _swerve_current_constants.Current_Limit_Steer,
            _swerve_current_constants.Steer_Current_Threshold,
            _swerve_current_constants.Steer_Current_Time
        };




        frc::PIDController _drive_pid_controller{SwerveConstants::DrivetrainConstants::DrivePIDConstants::Kp_Drive, SwerveConstants::DrivetrainConstants::DrivePIDConstants::Ki_Drive, SwerveConstants::DrivetrainConstants::DrivePIDConstants::Kd_Drive};
        frc::ProfiledPIDController<units::radians> _steer_pid_controller{SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kp_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Ki_Steer, SwerveConstants::DrivetrainConstants::SteerPIDConstants::Kd_Steer, 
            {SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_SPEED, SwerveConstants::DrivetrainConstants::SteerPIDConstants::MAX_ACCELERATION}};

        units::feet_per_second_t _GetWheelSpeed();
        units::inch_t _GetWheelPosition();
        units::degree_t _GetSteerAngle();

        frc::SimpleMotorFeedforward<units::meters> _drive_feed_forward{SwerveConstants::DrivetrainConstants::DriveFeedForwardConstants::S, SwerveConstants::DrivetrainConstants::DriveFeedForwardConstants::V, SwerveConstants::DrivetrainConstants::DriveFeedForwardConstants::A};
};

WPI_UNIGNORE_DEPRECATED

#endif