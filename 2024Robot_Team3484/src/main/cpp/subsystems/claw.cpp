#include "subsystems/claw.h"

ClawSubsystem::ClawSubsystem() {
_right_climb_motor.ConfigFactoryDefault();
_left_climb_motor.ConfigFactoryDefault();
_left_climb_motor.SetNeutralMode(motorcontrol::Brake);
_left_climb_motor.SetInverted(HookConstants::MOTOR_INVERTED);
_right_climb_motor.SetNeutralMode(motorcontrol::Brake);
_right_climb_motor.SetInverted(HookConstants::MOTOR_INVERTED);
}

void ClawSubsystem::Periodic(){}

bool ClawSubsystem::GetRightSensor(){
    return _right_motor_sensor.Get();
}

bool ClawSubsystem::GetLeftSensor(){
    return _left_motor_sensor.Get();
}

void ClawSubsystem::SetClawPower(double power){
    if (power < 0 && GetRightSensor()) 
        _right_climb_motor.Set(0);
    else
        _right_climb_motor.Set(power);
    if (power < 0 && GetLeftSensor()) 
    
        _left_climb_motor.Set(0);
    else    
        _left_climb_motor.Set(power);
    



}