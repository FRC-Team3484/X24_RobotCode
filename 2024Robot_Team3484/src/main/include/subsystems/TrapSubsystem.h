#ifndef TrapSubsystem_H
#define TrapSubsystem_H

#include <Constants.h>
#include <units/length.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>

class TrapSubsystem : public frc2::SubsystemBase {
    public: 
        TrapSubsystem(
            int extension_motor_can_id,
            int roller_motor_can_id,
            SC::SC_PIDConstants pidc,
            double pid_output_range_max,
            double pid_output_range_min
        );
        void Periodic() override;
        void SetRollerPower(double power);
        void SetPosition(units::inch_t position);
        bool AtPosition();
        units::inch_t GetExtension();
        units::feet_per_second_t GetExtensionVelocity();
        void OpenLoopTestMotors(double extension_motor, double roller_motor);

    private: 
        rev::CANSparkMax _extension_motor;
        rev::CANSparkMax _roller_motor;    

        rev::SparkRelativeEncoder* _extension_encoder;
        rev::SparkPIDController* _extension_pid_controller;

        frc::TrapezoidProfile<units::inches> _extension_trapezoid{{TrapConstants::MAX_VELOCITY, TrapConstants::MAX_ACCELERATION}};
        units::inch_t _target_position;
};

#endif