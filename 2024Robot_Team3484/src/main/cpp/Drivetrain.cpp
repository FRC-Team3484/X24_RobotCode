// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "ctre/phoenix/motorcontrol/NeutralMode.h"
#include "units/angle.h"

#include <frc2/command/Commands.h>

using namespace units;
using namespace ctre::phoenix::sensors;
using namespace ctre::phoenix;

Drive::Drive() {

    _drive_motor_BR.ConfigFactoryDefault();
    _drive_motor_BL.ConfigFactoryDefault();
    _drive_motor_FR.ConfigFactoryDefault();
    _drive_motor_FL.ConfigFactoryDefault();
    
    _drive_motor_BR.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    _drive_motor_BL.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    _drive_motor_FR.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    _drive_motor_FL.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

    _steer_motor_BR.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    _steer_motor_BL.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    _steer_motor_FR.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    _steer_motor_FL.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

     _drive_motor_BR.Follow(_drive_motor_FR);
     _drive_motor_BL.Follow(_drive_motor_FL);
    //  _drive_motor_BL.SetInverted(false);
    //  _drive_motor_FL.SetInverted(false);
    // //  _drive_motor_BR.SetInverted(false);
    //  _drive_motor_FR.SetInverted(false);

     m_drive.SetSafetyEnabled(false);

    _steer_motor_FL.ConfigFactoryDefault();
    _steer_motor_FL.SetNeutralMode(motorcontrol::Brake);
    _steer_motor_FL.ConfigSupplyCurrentLimit(_steer_current_limit);
    _steer_motor_FL.SetInverted(true);

    _steer_motor_FL.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255);
    _steer_motor_FL.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255);
    _steer_motor_FL.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255);
    _steer_motor_FL.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 200);
    

    _encoder_FL.ConfigFactoryDefault();
    _encoder_FL.SetPositionToAbsolute();
    _encoder_FL.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    _encoder_FL.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    _encoder_FL.ConfigMagnetOffset(-92.505);
    _encoder_FL.ConfigSensorDirection(false);
    _encoder_FL.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
    _encoder_FL.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 200);
    _steer_pid_controller_FL.EnableContinuousInput(-180_deg, 180_deg);
    
    _steer_motor_FR.ConfigFactoryDefault();
    _steer_motor_FR.SetNeutralMode(motorcontrol::Brake);
    _steer_motor_FR.ConfigSupplyCurrentLimit(_steer_current_limit);
    _steer_motor_FR.SetInverted(true);

    _steer_motor_FR.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255);
    _steer_motor_FR.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255);
    _steer_motor_FR.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255);
    _steer_motor_FR.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 200);
    

    _encoder_FR.ConfigFactoryDefault();
    _encoder_FR.SetPositionToAbsolute();
    _encoder_FR.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    _encoder_FR.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    _encoder_FR.ConfigMagnetOffset(-60.205);
    _encoder_FR.ConfigSensorDirection(false);
    _encoder_FR.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
    _encoder_FR.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 200);
    _steer_pid_controller_FR.EnableContinuousInput(-180_deg, 180_deg);

    _steer_motor_BR.ConfigFactoryDefault();
    _steer_motor_BR.SetNeutralMode(motorcontrol::Brake);
    _steer_motor_BR.ConfigSupplyCurrentLimit(_steer_current_limit);
    _steer_motor_BR.SetInverted(true);

    _steer_motor_BR.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255);
    _steer_motor_BR.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255);
    _steer_motor_BR.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255);
    _steer_motor_BR.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 200);
    

    _encoder_BR.ConfigFactoryDefault();
    _encoder_BR.SetPositionToAbsolute();
    _encoder_BR.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    _encoder_BR.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    _encoder_BR.ConfigMagnetOffset(-55.283);
    _encoder_BR.ConfigSensorDirection(false);
    _encoder_BR.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
    _encoder_BR.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 200);
    _steer_pid_controller_BR.EnableContinuousInput(-180_deg, 180_deg);
    
    _steer_motor_BL.ConfigFactoryDefault();
    _steer_motor_BL.SetNeutralMode(motorcontrol::Brake);
    _steer_motor_BL.ConfigSupplyCurrentLimit(_steer_current_limit);
    _steer_motor_BL.SetInverted(true);

    _steer_motor_BL.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255);
    _steer_motor_BL.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255);
    _steer_motor_BL.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255);
    _steer_motor_BL.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 200);
    

    _encoder_BL.ConfigFactoryDefault();
    _encoder_BL.SetPositionToAbsolute();
    _encoder_BL.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    _encoder_BL.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    _encoder_BL.ConfigMagnetOffset(160.654);
    _encoder_BL.ConfigSensorDirection(false);
    _encoder_BL.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20);
    _encoder_BL.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, 200);
    _steer_pid_controller_BL.EnableContinuousInput(-180_deg, 180_deg);
}

 



void Drive::Periodic() {


    double steer_output_FR = _steer_pid_controller_FR.Calculate(GetSteerAngleFR(), 0_rad);
    _steer_motor_FR.Set(steer_output_FR);

    double steer_output_FL = _steer_pid_controller_FL.Calculate(GetSteerAngleFL(), 0_rad);
    _steer_motor_FL.Set(steer_output_FL);

    double steer_output_BL = _steer_pid_controller_BL.Calculate(GetSteerAngleBL(), 0_rad);
    _steer_motor_BL.Set(steer_output_BL);

    double steer_output_BR = _steer_pid_controller_BR.Calculate(GetSteerAngleBR(), 0_rad);
    _steer_motor_BR.Set(steer_output_BR);

}


frc2::CommandPtr Drive::PseudoDriveCommand(std::function<double()> fwd,
                                           std::function<double()> rot) {
  return frc2::cmd::Run([this, fwd, rot] { m_drive.ArcadeDrive(fwd(), rot()); },
                        {this})
      .WithName("Psuedo Testing Arcade Drive");
}

frc2::CommandPtr Drive::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Drive::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}


degree_t Drive::GetSteerAngleFL() {
    return degree_t{_encoder_FL.GetAbsolutePosition()};
}
degree_t Drive::GetSteerAngleFR() {
    return degree_t{_encoder_FR.GetAbsolutePosition()};
}
degree_t Drive::GetSteerAngleBL() {
    return degree_t{_encoder_BL.GetAbsolutePosition()};
}
degree_t Drive::GetSteerAngleBR() {
    return degree_t{_encoder_BR.GetAbsolutePosition()};
}
