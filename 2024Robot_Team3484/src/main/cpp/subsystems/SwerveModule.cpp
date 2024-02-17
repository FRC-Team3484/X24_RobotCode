#include "subsystems/SwerveModule.h"

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <wpi/deprecated.h>

WPI_IGNORE_DEPRECATED
#include <frc/geometry/Rotation2d.h>

using namespace frc;
using namespace units;
// using namespace ctre::phoenix6;
using namespace ctre;
using namespace SC;
using namespace SwerveConstants::DrivetrainConstants;
using namespace ctre::phoenix::sensors;
using namespace ctre::phoenix;

SwerveModule::SwerveModule(SC_SwerveConfigs corner) 
        : _drive_motor(corner.CAN_ID),
            _steer_motor(corner.SteerMotorPort),
            _steer_encoder(corner.EncoderPort)
        {

    // configs::CurrentLimitsConfigs drive_motor_current_limit{};
    // drive_motor_current_limit.SupplyCurrentThreshold = DRIVE_CURRENT_THRESHOLD;
    // drive_motor_current_limit.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT;
    // drive_motor_current_limit.SupplyTimeThreshold = DRIVE_CURRENT_TIME;
    // drive_motor_current_limit.SupplyCurrentLimitEnable = true;

    // _drive_motor_config.CurrentLimits = drive_motor_current_limit;

    // _drive_motor.GetConfigurator().Apply(_drive_motor_config);
    // ResetEncoder();
    // SetCoastMode();

    _drive_motor.ConfigFactoryDefault();
    _drive_motor.ConfigSupplyCurrentLimit(_drive_currrent_limit);
    ResetEncoder();

    // Change to Phoenix 5
    // configs::CurrentLimitsConfigs steer_motor_current_limit{};
    // steer_motor_current_limit.SupplyCurrentThreshold = STEER_CURRENT_THRESHOLD;
    // steer_motor_current_limit.SupplyCurrentLimit = STEER_CURRENT_LIMIT;
    // steer_motor_current_limit.SupplyTimeThreshold = STEER_CURRENT_TIME;
    // steer_motor_current_limit.SupplyCurrentLimitEnable = true;

    // configs::TalonFXConfiguration steer_motor_config{};
    // steer_motor_config.CurrentLimits = steer_motor_current_limit;
    // steer_motor_config.MotorOutput.Inverted = STEER_MOTOR_REVERSED;
    // steer_motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    // _steer_motor.GetConfigurator().Apply(steer_motor_config);
    _steer_motor.ConfigFactoryDefault();
    _steer_motor.SetNeutralMode(motorcontrol::Brake);
    _steer_motor.ConfigSupplyCurrentLimit(_steer_current_limit);
    _steer_motor.SetInverted(_swerve_current_constants.Steer_Motor_Reversed);

    _steer_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255);
    _steer_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255);
    _steer_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255);
    _steer_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 200);

    _steer_encoder.ConfigFactoryDefault();
    _steer_encoder.SetPositionToAbsolute();
    _steer_encoder.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    _steer_encoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    _steer_encoder.ConfigMagnetOffset(corner.EncoderOffset);
    _steer_encoder.ConfigSensorDirection(_swerve_current_constants.Encoder_Reversed);
    _steer_encoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
    _steer_encoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 200);

    // Change all the way to Cancoder
    // configs::MagnetSensorConfigs encoder_magnet_config{};
    // encoder_magnet_config.MagnetOffset = ENCODER_OFFSET[module_location] / 360.0;
    // encoder_magnet_config.AbsoluteSensorRange = signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
    // encoder_magnet_config.SensorDirection = ENCODER_REVERSED;

    // configs::CANcoderConfiguration encoder_config{};
    // encoder_config.MagnetSensor = encoder_magnet_config;

    // _steer_encoder.GetConfigurator().Apply(encoder_config);

    _steer_pid_controller.EnableContinuousInput(-180_deg, 180_deg);

    //SetDesiredState({0_mps, GetState().angle}, true);
}

void SwerveModule::SetDesiredState(SwerveModuleState state, bool open_loop, bool optimize) {
    Rotation2d encoder_rotation{_GetSteerAngle()};

    //If the wheel needs to rotate over 90 degrees, rotate the other direction and flip the output
    //This prevents the wheel from ever needing to rotate more than 90 degrees
    if (optimize)
        state = SwerveModuleState::Optimize(state, encoder_rotation);

    //Scale the wheel speed down by the cosine of the angle error
    //This prevents the wheel from accelerating before it has a chance to face the correct direction
    state.speed *= (state.angle - encoder_rotation).Cos();

    //In open loop, treat speed as a percent power
    //In closed loop, try to hit the acutal speed
    if (open_loop) {
        _drive_motor.Set(state.speed / MAX_WHEEL_SPEED *.2);
    } else {
        volt_t drive_output = volt_t{_drive_pid_controller.Calculate(meters_per_second_t{_GetWheelSpeed()}.value(), state.speed.value())};
        volt_t drive_feed_forward = _drive_feed_forward.Calculate(state.speed);
        _drive_motor.SetVoltage(drive_output + drive_feed_forward);
    }

    double steer_output = _steer_pid_controller.Calculate(_GetSteerAngle(), state.angle.Radians());
    _steer_motor.Set(steer_output);
}

SwerveModuleState SwerveModule::GetState() {
    return {_GetWheelSpeed(), _GetSteerAngle()};
}

SwerveModulePosition SwerveModule::GetPosition() {
    return {_GetWheelPosition(), _GetSteerAngle()};
}

feet_per_second_t SwerveModule::_GetWheelSpeed() {
    // return WHEEL_RADIUS * radians_per_second_t{_drive_motor.GetVelocity().GetValue() / DRIVE_GEAR_RATIO} / 1_rad;
    return WHEEL_RADIUS * radians_per_second_t{_drive_motor.GetSelectedSensorVelocity() * 10.0 * 360.0_deg_per_s / DRIVE_TICKS_PER_REVOLUTION / DRIVE_GEAR_RATIO} / 1_rad;
}

inch_t SwerveModule::_GetWheelPosition() {
    // return WHEEL_RADIUS * radian_t{_drive_motor.GetPosition().GetValue() / DRIVE_GEAR_RATIO} / 1_rad;
    return WHEEL_RADIUS * radian_t{_drive_motor.GetSelectedSensorPosition() * 360.0_deg / DRIVE_TICKS_PER_REVOLUTION / DRIVE_GEAR_RATIO} / 1_rad;
}

degree_t SwerveModule::_GetSteerAngle() {
    // return _steer_encoder.GetAbsolutePosition().GetValue();
    return degree_t{_steer_encoder.GetAbsolutePosition()};
}

void SwerveModule::StopMotors() {
    _drive_motor.Set(0);
    _steer_motor.Set(0);
}

void SwerveModule::ResetEncoder() {
    // _drive_motor.SetPosition(0_deg);
    _drive_motor.SetSelectedSensorPosition(0);
}

// Change to Set Coast Mode
void SwerveModule::SetCoastMode() {
    _drive_motor.SetNeutralMode(motorcontrol::Coast);
}

void SwerveModule::SetBrakeMode() {
    // _drive_motor_config.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    // _drive_motor.GetConfigurator().Apply(_drive_motor_config);
    _drive_motor.SetNeutralMode(motorcontrol::Brake);
}
WPI_UNIGNORE_DEPRECATED
