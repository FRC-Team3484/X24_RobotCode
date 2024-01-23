#include "subsystems/ClawSubsystem.h"

ClawSubsystem::ClawSubsystem(
    int left_motor_can_id,
    int right_motor_can_id,
    int left_sensor_di_ch,
    int right_sensor_di_ch

    ) : 
        _left_climb_motor{left_motor_can_id},
        _right_climb_motor{right_motor_can_id},
        _left_motor_sensor{left_sensor_di_ch},
        _right_motor_sensor{right_sensor_di_ch}
    {

    _right_climb_motor.ConfigFactoryDefault();
    _left_climb_motor.ConfigFactoryDefault();

    _left_climb_motor.SetNeutralMode(motorcontrol::Brake);
    _right_climb_motor.SetNeutralMode(motorcontrol::Brake);

    _left_climb_motor.SetInverted(ClawConstants::MOTOR_INVERTED);
    _right_climb_motor.SetInverted(ClawConstants::MOTOR_INVERTED);
}

void ClawSubsystem::Periodic() {}

bool ClawSubsystem::GetLeftSensor() {
    // Returns the value of the left limit switch
    return _left_motor_sensor.Get();
}

bool ClawSubsystem::GetRightSensor() {
    // Returns the value of the right limit sensor
    return _right_motor_sensor.Get();
}

void ClawSubsystem::SetClawPower(double power) {
    // Sets the claw power
    if (power < 0 && GetRightSensor()) 
        _right_climb_motor.Set(ClawConstants::MOTOR_STOP);
    else
        _right_climb_motor.Set(power);

    if (power < 0 && GetLeftSensor()) 
        _left_climb_motor.Set(ClawConstants::MOTOR_STOP);
    else    
        _left_climb_motor.Set(power);
}