#ifndef INTAKE_H
#define INTAKE_H


#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
//#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/angle.h>

#include <Constants.h>

class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem(
            int PIVOT_MOTOR_CAN_ID, 
            int DRIVE_MOTOR_CAN_ID, 
            int PIECE_SENSOR_DI_CH, 
            int ARM_SENSOR_DI_CH,
            
            double PID_P,
            int PID_I,
            int PID_D,
            int PID_IZ_ZONE,
            int PID_FF,
            int PID_OUTPUTRANGE_MIN,
            int PID_OUTPUTRANGE_MAX
        );

        void Periodic() override;
        void SetIntakeAngle(units::degree_t angle);
        void SetRollerPower(double power);
        bool HasPiece();
        bool ArmExtended();
        units::turn_t GetIntakePosition();
        units::revolutions_per_minute_t GetEncoderVelocity();

    private:
        bool _arm_sensor_hit = false;
        
        rev::CANSparkMax _pivot_motor;
        rev::CANSparkMax _drive_motor;

        frc::DigitalInput _piece_sensor;
        frc::DigitalInput _arm_sensor;

        rev::SparkRelativeEncoder* _pivot_encoder;
        rev::SparkPIDController* _pivot_pid_controller;

        frc::TrapezoidProfile<units::degrees> _intake_trapezoid{{IntakeConstants::MAX_VELOCITY, IntakeConstants::MAX_ACCELERATION}};
        units::degree_t _target_position;
};


#endif