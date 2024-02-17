#include "subsystems/ClimberSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <wpi/deprecated.h>

WPI_IGNORE_DEPRECATED

using namespace frc;

ClimberSubsystem::ClimberSubsystem(
    int left_motor_can_id,
    int right_motor_can_id,
    int left_sensor_di_ch,
    int right_sensor_di_ch

    ) : 
        _left_climber_motor{left_motor_can_id},
        _right_climber_motor{right_motor_can_id},
        _left_motor_sensor{left_sensor_di_ch},
        _right_motor_sensor{right_sensor_di_ch}
    {

    _left_climber_motor.ConfigFactoryDefault();
    _right_climber_motor.ConfigFactoryDefault();


    _left_climber_motor.SetInverted(ClimberConstants::MOTOR_INVERTED);
    _right_climber_motor.SetInverted(ClimberConstants::MOTOR_INVERTED);
}

void ClimberSubsystem::Periodic() {
    #ifdef EN_DIAGNOSTICS
        SmartDashboard::PutBoolean("Left Limit Sensor", GetLeftSensor());
        SmartDashboard::PutBoolean("Right Limit Sensor", GetRightSensor());
        SmartDashboard::PutNumber("Climber Power Right", _right_climber_motor.Get());
        SmartDashboard::PutNumber("Climber Power Left", _left_climber_motor.Get());
    #endif

}

bool ClimberSubsystem::GetLeftSensor() {
    // Returns the value of the left limit switch
    return _left_motor_sensor.Get();
}

bool ClimberSubsystem::GetRightSensor() {
    // Returns the value of the right limit sensor
    return _right_motor_sensor.Get();
}

void ClimberSubsystem::SetClimberPower(double power) {
    // Sets the Climber power
    if (power < 0 && GetRightSensor()) 
        _right_climber_motor.Set(ClimberConstants::MOTOR_STOP);
    else
        _right_climber_motor.Set(power);

    if (power < 0 && GetLeftSensor()) 
        _left_climber_motor.Set(ClimberConstants::MOTOR_STOP);
    else    
        _left_climber_motor.Set(power);
}

void ClimberSubsystem::OpenLoopTestMotors(double power_left, double power_right) {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {
        _left_climber_motor.Set(power_left);
        _right_climber_motor.Set(power_right);
    }
}

WPI_UNIGNORE_DEPRECATED