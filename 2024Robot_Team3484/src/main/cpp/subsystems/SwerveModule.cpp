#include "subsystems/SwerveModule.h"

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <frc/geometry/Rotation2d.h>



using namespace SwerveConstants::DrivetrainConstants;
using namespace units;
using namespace frc;



SwerveModule::SwerveModule(const int module_location) 
    :_drive_motor(DRIVE_MOTOR_PORTS[module_location]),
     _steer_motor(STEER_MOTOR_PORTS[module_location]),
     _steer_encoder(ENCODER_PORTS[module_location])
{
    _drive_motor.ConfigFactoryDefault();
    _drive_motor.ConfigSupplyCurrentLimit(DRIVE_CURRENT_LIMIT);
    ResetEncoder();

    _steer_motor.ConfigFactoryDefault();
    _drive_motor.ConfigSupplyCurrentLimit(STEER_CURRENT_LIMIT);
    // _steer_motor.SetNeutralMode(motorcontrol::Brake);
    _steer_motor.SetInverted(STEER_MOTOR_REVERSED);

    _steer_encoder.ConfigFactoryDefault();
    _steer_encoder.SetPositionToAbsolute();
    _steer_encoder.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    _steer_encoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    _steer_encoder.ConfigMagnetOffset(ENCODER_OFFSET[module_location]);
    _steer_encoder.ConfigSensorDirection(ENCODER_REVERSED);

    _steer_pid_controller.EnableContinuousInput(-180_deg, 180_deg);

    SetDesiredState({0_mps, GetState().angle}, true);
}

void SwerveModule::SetDesiredState(SwerveModuleState state, bool open_loop, bool optimize) {
    // State is a velocity annd a direction
    Rotation2d encoder_rotation{_GetSteerAngle()}; // what current durection (angle)


    if (optimize) {
        //Foor the Brake
        state = SwerveModuleState::Optimize(state, encoder_rotation);
    }

    state.speed *= (state.angle - encoder_rotation).Cos(); //difference of yhat and y
    //dampener when changing; prevents wheel from accelarating when facing the proper direction

    if (open_loop) {
        _drive_motor.Set(ControlMode::PercentOutput, state.speed / MAX_WHEEL_SPEED);
        // for converting open loops to straight power level
    } else {
        const volt_t drive_PID_output = volt_t{_drive_pid_controller.Calculate(meters_per_second_t{_GetWheelSpeed()}.value(), state.speed.value())};
        const volt_t drive_feed_forward = _drive_feed_forward.Calculate(state.speed); //calculator
        //calculates power and velocity relation
        _drive_motor.SetVoltage(drive_PID_output + drive_feed_forward);
    }

    const double steer_output = _steer_pid_controller.Calculate(_GetSteerAngle(), state.angle.Radians());
    _steer_motor.Set(ControlMode::PercentOutput, steer_output);
}

void SwerveModule::ResetEncoder() {
    _drive_motor.SetSelectedSensorPosition(0);
}

void SwerveModule::StopMotors() {
    _drive_motor.Set(0);
    _steer_motor.Set(0);
}

void SwerveModule::ResetEncoder() {
    _drive_motor.SetSelectedSensorPosition(0);
}

void SwerveModule::SetCoastMode() {
    _drive_motor.SetNeutralMode(motorcontrol::Coast);
}

void SwerveModule::SetBrakeMode() {
    _drive_motor.SetNeutralMode(motorcontrol::Brake);
}

SwerveModuleState SwerveModule::GetState() {
    return {_GetWheelSpeed(), _GetSteerAngle()};
}

SwerveModulePosition SwerveModule::GetPosition() { //used by odometry
    return {_GetWheelPosition(), _GetSteerAngle()};
}

feet_per_second_t SwerveModule::_GetWheelSpeed() {
    return WHEEL_RADIUS * radians_per_second_t{_drive_motor.GetSelectedSensorVelocity() * 10 * 360.0_deg_per_s / DRIVE_TICKS_PER_REVOLUTION / DRIVE_GEAR_RATIO}/ 1_rad;
}

inch_t SwerveModule::_GetWheelPosition() {
    return WHEEL_RADIUS * radian_t{_drive_motor.GetSelectedSensorPosition() * 360.0_deg / DRIVE_TICKS_PER_REVOLUTION / DRIVE_GEAR_RATIO} / 1_rad;
}

degree_t SwerveModule::_GetSteerAngle() {
    return degree_t{_steer_encoder.GetAbsolutePosition()};
}