#ifndef CLIMBERSUBSYSTEM_H
#define CLIMBERSUBSYSTEM_H

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
// #include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#include <wpi/deprecated.h>

WPI_IGNORE_DEPRECATED

#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

class ClimberSubsystem : public frc2::SubsystemBase {
    public: 
        ClimberSubsystem(
            int left_motor_can_id,
            int right_motor_can_id,
            int left_sensor_di_ch,
            int right_sensor_di_ch
        );
        void Periodic() override;
        void SetClimberPower(double power);
        bool GetLeftSensor();
        bool GetRightSensor();
        void OpenLoopTestMotors(double power_left, double power_right);

    private: 
        ctre::phoenix::motorcontrol::can::WPI_TalonFX _left_climber_motor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX _right_climber_motor;


        frc::DigitalInput _left_motor_sensor;
        frc::DigitalInput _right_motor_sensor;
        bool _left_homed = true;
        bool _right_homed = true;
    
};

WPI_UNIGNORE_DEPRECATED

#endif