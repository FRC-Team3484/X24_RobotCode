#include "subsystems/SwerveModule.h"

using namespace DrivetrainConstants;

SwerveModule::SwerveModule(const int module_location) 
    :_drive_motor(DRIVE_MOTOR_PORTS[module_location]),
     _steer_motor(STEER_MOTOR_PORTS[module_location]),
     _steer_encoder(ENCODER_PORTS[module_location])
{
    _drive_motor.ConfigFactoryDefault();
    _drive_motor.SetNeutralMode(motorcontrol::Coast);
    ResetEncoder();

    _steer_motor.ConfigFactoryDefault();
    _steer_motor.SetNeutralMode(motorcontrol::Coast);
    _steer_motor.SetInverted(STEER_MOTOR_REVERSED[module_location]);
    
    _steer_encoder.ConfigFactoryDefault();
    _steer_encoder.SetPositionToAbsolute();
    _steer_encoder.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    _steer_encoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    _steer_encoder.ConfigMagnetOffset(ENCODER_OFFSET[module_location]);
    _steer_encoder.ConfigSensorDirection(ENCODER_REVERSED[module_location]);

    _steer_pid_controller.EnableContinuousInput(-180_deg, 180_deg);

    SetDesiredState({0_mps, GetState().angle}, true);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state, bool open_loop) {
    frc::Rotation2d encoder_rotation{_GetSteerAngle()};

    frc::SwerveModuleState optimized_state = frc::SwerveModuleState::Optimize(state, encoder_rotation);

    optimized_state.speed *= (optimized_state.angle - encoder_rotation).Cos();

    if (open_loop) {
        _drive_motor.Set(ControlMode::PercentOutput, 0.3* optimized_state.speed / MAX_LINEAR_SPEED);
    } else {
        const units::volt_t drive_output = units::volt_t{_drive_pid_controller.Calculate(units::meters_per_second_t{_GetWheelSpeed()}.value(), optimized_state.speed.value())};
        const units::volt_t drive_feed_forward = _dirve_feed_forward.Calculate(optimized_state.speed);
        _drive_motor.SetVoltage(drive_output + drive_feed_forward);
    }

    const double steer_output = _steer_pid_controller.Calculate(_GetSteerAngle(), optimized_state.angle.Radians());
    _steer_motor.Set(ControlMode::PercentOutput, steer_output);
}

void SwerveModule::ResetEncoder() {
    _drive_motor.SetSelectedSensorPosition(0);
}

frc::SwerveModuleState SwerveModule::GetState() {
    return {_GetWheelSpeed(), _GetSteerAngle()};
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    return {_GetWheelPosition(), _GetSteerAngle()};
}

units::feet_per_second_t SwerveModule::_GetWheelSpeed() {
    return WHEEL_RADIUS * units::radians_per_second_t{_drive_motor.GetSelectedSensorVelocity() * 10 * 360.0_deg_per_s / DRIVE_TICKS_PER_REVOLUTION / DRIVE_GEAR_RATIO}/ 1_rad;
}

units::inch_t SwerveModule::_GetWheelPosition() {
    return WHEEL_RADIUS * units::radian_t{_drive_motor.GetSelectedSensorPosition() * 360.0_deg / DRIVE_TICKS_PER_REVOLUTION / DRIVE_GEAR_RATIO} / 1_rad;
}

units::degree_t SwerveModule::_GetSteerAngle() {
    return units::degree_t{_steer_encoder.GetAbsolutePosition()};
}