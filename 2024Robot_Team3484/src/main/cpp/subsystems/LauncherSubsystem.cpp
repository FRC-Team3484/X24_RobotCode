// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or Launcherare it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LauncherSubsystem.h"

#include <numbers>

using namespace LauncherConstants;
using namespace rev;

LauncherSubsystem::LauncherSubsystem(
        int MOTOR_LEFT_CAN_ID, 
        int MOTOR_RIGHT_CAN_ID,
        double P_Launcher,
        double I_Launcher,
        double D_Launcher,
        double FF_Launcher,
        double RPM_Window_Launcher 
    ):
    _left_motor{MOTOR_LEFT_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
    _right_motor{MOTOR_RIGHT_CAN_ID, rev::CANSparkMax::MotorType::kBrushless}
    {
    Launcher_Encoder_Left = new SparkRelativeEncoder(_left_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 ));
    Launcher_Encoder_Right = new SparkRelativeEncoder(_right_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 ));

    Launcher_m_Left_pidController = new SparkPIDController(_left_motor.GetPIDController());
    Launcher_m_Right_pidController = new SparkPIDController(_right_motor.GetPIDController());

    Launcher_m_Left_pidController->SetFeedbackDevice(*Launcher_Encoder_Left);
    Launcher_m_Right_pidController->SetFeedbackDevice(*Launcher_Encoder_Right);

    _left_motor.RestoreFactoryDefaults();
    _right_motor.RestoreFactoryDefaults();

    _left_motor.SetInverted(MOTOR_INVERTED);
    _right_motor.SetInverted(!MOTOR_INVERTED);

    Launcher_m_Left_pidController->SetP(P_Launcher);
    Launcher_m_Left_pidController->SetI(I_Launcher);
    Launcher_m_Left_pidController->SetD(D_Launcher);
    Launcher_m_Left_pidController->SetIZone(0);
    Launcher_m_Left_pidController->SetFF(FF_Launcher);
    Launcher_m_Left_pidController->SetOutputRange(0, 1);

    Launcher_m_Right_pidController->SetP(P_Launcher);
    Launcher_m_Right_pidController->SetD(D_Launcher);
    Launcher_m_Right_pidController->SetI(I_Launcher);
    Launcher_m_Right_pidController->SetIZone(0);
    Launcher_m_Right_pidController->SetFF(FF_Launcher);
    Launcher_m_Right_pidController->SetOutputRange(0, 1);
}

void LauncherSubsystem::setLauncherRPM(units::revolutions_per_minute_t speed){
    _target_speed = speed.value();
}

void LauncherSubsystem::Periodic() {
    Launcher_m_Right_pidController->SetReference(_target_speed, rev::CANSparkMax::ControlType::kVelocity);
    Launcher_m_Left_pidController->SetReference(_target_speed, rev::CANSparkMax::ControlType::kVelocity);
}

bool LauncherSubsystem::atTargetRPM(){
 return std::abs(Launcher_Encoder_Left->GetVelocity()-_target_speed) < RPM_Window_Launcher && std::abs(Launcher_Encoder_Right->GetVelocity()-_target_speed) < RPM_Window_Launcher;   
}