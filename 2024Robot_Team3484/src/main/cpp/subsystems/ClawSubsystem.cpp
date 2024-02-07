#include "subsystems/ClawSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

ClawSubsystem::ClawSubsystem(
    int left_motor_can_id,
    int right_motor_can_id,
    int left_sensor_di_ch,
    int right_sensor_di_ch

    ) : 
        _left_climb_motor{left_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
        _right_climb_motor{right_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
        _left_motor_sensor{left_sensor_di_ch},
        _right_motor_sensor{right_sensor_di_ch}
    {

    _right_climb_motor.RestoreFactoryDefaults();
    _left_climb_motor.RestoreFactoryDefaults();

    _left_climb_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    _right_climb_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    _left_climb_motor.SetInverted(ClawConstants::MOTOR_INVERTED);
    _right_climb_motor.SetInverted(ClawConstants::MOTOR_INVERTED);
}

void ClawSubsystem::Periodic() {
    #ifdef EN_DIAGNOSTICS
        SmartDashboard::PutBoolean("Left Limit Sensor", GetLeftSensor());
        SmartDashboard::PutBoolean("Right Limit Sensor", GetRightSensor());
        SmartDashboard::PutNumber("Claw Power Right", _right_climb_motor.Get());
        SmartDashboard::PutNumber("Claw Power Left", _left_climb_motor.Get());
    #endif

}

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