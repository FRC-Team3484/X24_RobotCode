#ifndef claw_h
#define claw_h

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>


// 

class ClawSubsystem : public frc2::SubsystemBase {
    public: 
        ClawSubsystem();
        void Periodic() override;
        void SetClawPower(double power);
        bool GetLeftSensor();
        bool GetRightSensor();

    private: 
        WPI_TalonSRX _left_climb_motor{HookConstants::LEFT_MOTOR_CAN_ID};
        WPI_TalonSRX _right_climb_motor{HookConstants::RIGHT_MOTOR_CAN_ID};
        frc::DigitalInput _left_motor_sensor{HookConstants::LEFT_SENSOR_DI_CH};
        frc::DigitalInput _right_motor_sensor{HookConstants::RIGHT_SENSOR_DI_CH};
    


};
#endif