#ifndef CLIMBERSUBSYSTEM_H
#define CLIMBERSUBSYSTEM_H

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

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

    private: 
        rev::CANSparkMax _left_climber_motor;
        rev::CANSparkMax _right_climber_motor;

        frc::DigitalInput _left_motor_sensor;
        frc::DigitalInput _right_motor_sensor;
    
};
#endif