// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <numbers>

using namespace ShooterConstants;

ShooterSubsystem::ShooterSubsystem() {

    SH_Left_motor.RestoreFactoryDefaults();
    SH_Right_motor.RestoreFactoryDefaults();

    SH_Left_motor.SetInverted(MOTER_LEFT_INVERTED);
    SH_Right_motor.SetInverted(!MOTER_LEFT_INVERTED);

    SH_m_Left_pidController.SetP(P);
    SH_m_Left_pidController.SetI(I);
    SH_m_Left_pidController.SetD(D);
    SH_m_Left_pidController.SetIZone(0);
    SH_m_Left_pidController.SetFF(FF);
    SH_m_Left_pidController.SetOutputRange(0, 1);

    SH_m_Right_pidController.SetP(P);
    SH_m_Right_pidController.SetI(I);
    SH_m_Right_pidController.SetD(D);
    SH_m_Right_pidController.SetIZone(0);
    SH_m_Right_pidController.SetFF(FF);
    SH_m_Right_pidController.SetOutputRange(0, 1);
}

void ShooterSubsystem::setSHRPM(units::revolutions_per_minute_t speed){
    _speed = speed.value();
}

void ShooterSubsystem::Periodic() {
    SH_m_Right_pidController.SetReference(_speed, rev::CANSparkMax::ControlType::kVelocity);
    SH_m_Left_pidController.SetReference(_speed, rev::CANSparkMax::ControlType::kVelocity);
}

bool ShooterSubsystem::atPoint(){
 return std::abs(SH_Encoder_Left.GetVelocity()-_speed) < RPM_Window and std::abs(SH_Encoder_Right.GetVelocity()-_speed) < RPM_Window;   
}