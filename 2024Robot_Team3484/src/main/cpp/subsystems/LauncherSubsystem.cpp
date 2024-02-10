// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or Launcherare it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LauncherSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <numbers>

using namespace LauncherConstants;
using namespace rev;
using namespace frc;

LauncherSubsystem::LauncherSubsystem(
        int left_motor_can_id,
        int right_motor_can_id,
        SC::SC_PIDConstants pidc,
        double rpm_window
    ):
    _left_motor{left_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
    _right_motor{right_motor_can_id, rev::CANSparkMax::MotorType::kBrushless}
    {
        _rpm_window = rpm_window;

        _left_launcher_encoder = new SparkRelativeEncoder(_left_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 ));
        _right_launcher_encoder = new SparkRelativeEncoder(_right_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 ));

    
        _left_launcher_pid_controller = new SparkPIDController(_left_motor.GetPIDController());
    
        _right_launcher_pid_controller = new SparkPIDController(_right_motor.GetPIDController());
        
    if (_left_launcher_pid_controller !=NULL){    
        _left_launcher_pid_controller->SetFeedbackDevice(*_left_launcher_encoder);
    }
    if (_right_launcher_pid_controller !=NULL){
        _right_launcher_pid_controller->SetFeedbackDevice(*_right_launcher_encoder);
    }
    _left_motor.RestoreFactoryDefaults();
    _right_motor.RestoreFactoryDefaults();

    _left_motor.SetInverted(MOTOR_INVERTED);
    _right_motor.SetInverted(!MOTOR_INVERTED);
    if (_left_launcher_pid_controller !=NULL){
        _left_launcher_pid_controller->SetP(pidc.Kp);
        _left_launcher_pid_controller->SetI(pidc.Ki);
        _left_launcher_pid_controller->SetD(pidc.Kd);
        _left_launcher_pid_controller->SetIZone(0);
        _left_launcher_pid_controller->SetFF(pidc.Kf);
        _left_launcher_pid_controller->SetOutputRange(0, 1);
        }
    if (_right_launcher_pid_controller !=NULL){
        _right_launcher_pid_controller->SetP(pidc.Kp);
        _right_launcher_pid_controller->SetI(pidc.Ki);
        _right_launcher_pid_controller->SetD(pidc.Kd);
        _right_launcher_pid_controller->SetIZone(0);
        _right_launcher_pid_controller->SetFF(pidc.Kf);
        _right_launcher_pid_controller->SetOutputRange(0, 1);
    }
}

void LauncherSubsystem::setLauncherRPM(units::revolutions_per_minute_t speed){
    _target_speed = speed.value()*GEAR_RATIO;
}

void LauncherSubsystem::Periodic() {
    #ifdef EN_DIAGNOSTICS
        SmartDashboard::PutNumber("Motor Speed Left (RPM)", _left_launcher_encoder->GetVelocity()/GEAR_RATIO);
        SmartDashboard::PutNumber("Motor Speed Right (RPM)", _right_launcher_encoder->GetVelocity()/GEAR_RATIO);
    #endif
    _counter_not_null_right = 0;
    _counter_not_null_left = 0;

    if (_left_launcher_pid_controller !=NULL){
        _left_launcher_pid_controller->SetReference(_target_speed, rev::CANSparkMax::ControlType::kVelocity);
        _counter_not_null_left++;
    }
    if (_right_launcher_pid_controller !=NULL){
        _right_launcher_pid_controller->SetReference(_target_speed, rev::CANSparkMax::ControlType::kVelocity);
        _counter_not_null_right++;
    }
}

bool LauncherSubsystem::atTargetRPM(){
    if (_counter_not_null_left + _counter_not_null_right == 2){
        return std::abs(_left_launcher_encoder->GetVelocity()-_target_speed) < _rpm_window && std::abs(_right_launcher_encoder->GetVelocity()-_target_speed) < _rpm_window;   
    }
    else if (_counter_not_null_left == 1) {
        return std::abs(_left_launcher_encoder->GetVelocity()-_target_speed) < _rpm_window;   
    }
    else if (_counter_not_null_right == 1) {
        return std::abs(_right_launcher_encoder->GetVelocity()-_target_speed) < _rpm_window;   
    }
    else {
        return false;
    }
}

#ifdef EN_TESTING
void LauncherSubsystem::OpenLoopTestMotors(double power_left, double power_right) {
    _left_motor.Set(power_left);
    _right_motor.Set(power_right);
}

#endif
