// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or Launcherare it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LauncherSubsystem.h"

#include <numbers>

using namespace LauncherConstants;
using namespace rev;

LauncherSubsystem::LauncherSubsystem(
        int CAN_ID_Left,
        int CAN_ID_Right,
        SC::SC_PIDConstants pidc,
        double RPM_InRangeWindow
    ):
    _left_motor{CAN_ID_Left, rev::CANSparkMax::MotorType::kBrushless},
    _right_motor{CAN_ID_Right, rev::CANSparkMax::MotorType::kBrushless}
    {
        Launcher_Encoder_Left = new SparkRelativeEncoder(_left_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 ));
        Launcher_Encoder_Right = new SparkRelativeEncoder(_right_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 ));

    
        Launcher_m_Left_pidController = new SparkPIDController(_left_motor.GetPIDController());
    
        Launcher_m_Right_pidController = new SparkPIDController(_right_motor.GetPIDController());
        
    if (Launcher_m_Left_pidController !=NULL){    
        Launcher_m_Left_pidController->SetFeedbackDevice(*Launcher_Encoder_Left);
    }
    if (Launcher_m_Right_pidController !=NULL){
        Launcher_m_Right_pidController->SetFeedbackDevice(*Launcher_Encoder_Right);
    }
    _left_motor.RestoreFactoryDefaults();
    _right_motor.RestoreFactoryDefaults();

    _left_motor.SetInverted(MOTOR_INVERTED);
    _right_motor.SetInverted(!MOTOR_INVERTED);
    if (Launcher_m_Left_pidController !=NULL){
        Launcher_m_Left_pidController->SetP(pidc.Kp);
        Launcher_m_Left_pidController->SetI(pidc.Ki);
        Launcher_m_Left_pidController->SetD(pidc.Kd);
        Launcher_m_Left_pidController->SetIZone(0);
        Launcher_m_Left_pidController->SetFF(pidc.Kf);
        Launcher_m_Left_pidController->SetOutputRange(0, 1);
        }
    if (Launcher_m_Right_pidController !=NULL){
        Launcher_m_Right_pidController->SetP(pidc.Kp);
        Launcher_m_Right_pidController->SetI(pidc.Ki);
        Launcher_m_Right_pidController->SetD(pidc.Kd);
        Launcher_m_Right_pidController->SetIZone(0);
        Launcher_m_Right_pidController->SetFF(pidc.Kf);
        Launcher_m_Right_pidController->SetOutputRange(0, 1);
    }
}

void LauncherSubsystem::setLauncherRPM(units::revolutions_per_minute_t speed){
    _target_speed = speed.value()*GEAR_RATIO;
}

void LauncherSubsystem::Periodic() {
    _counter_null_right = 0;
    _counter_null_left = 0;

    if (Launcher_m_Left_pidController !=NULL){
        Launcher_m_Left_pidController->SetReference(_target_speed, rev::CANSparkMax::ControlType::kVelocity);
        _counter_null_left++;
    }
     if (Launcher_m_Right_pidController !=NULL){
        Launcher_m_Right_pidController->SetReference(_target_speed, rev::CANSparkMax::ControlType::kVelocity);
        _counter_null_right++;
     }
}

bool LauncherSubsystem::atTargetRPM(){
    if (_counter_null_left + _counter_null_right == 0){
        return std::abs(Launcher_Encoder_Left->GetVelocity()-_target_speed) < RPM_WINDOW_RANGE && std::abs(Launcher_Encoder_Right->GetVelocity()-_target_speed) < RPM_WINDOW_RANGE;   
    }
    else if (_counter_null_left == 1) {
        return std::abs(Launcher_Encoder_Left->GetVelocity()-_target_speed) < RPM_WINDOW_RANGE;   
    }
    else if (_counter_null_right == 1) {
        return std::abs(Launcher_Encoder_Right->GetVelocity()-_target_speed) < RPM_WINDOW_RANGE;   
    }
    if (_counter_null_left + _counter_null_right == 2){
        return false;
    }
}