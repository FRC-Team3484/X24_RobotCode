#ifndef INTAKE_H
#define INTAKE_H

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
//#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>

#include <units/angle.h>

#include <Constants.h>
#include "FRC3484_Lib/utils/SC_Datatypes.h"

//Amp
#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"


class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem(
            int pivot_motor_can_id, 
            int drive_motor_can_id, 
            int piece_sensor_di_ch,
            int arm_sensor_di_ch,
            SC::SC_PIDConstants pivot_pidc,
            double pid_output_range_max,
            double pid_output_range_min,
            int amp_motor_id
            /*Amp Stuff*/
        );

        void Periodic() override;
        void SetIntakeAngle(units::degree_t angle, bool force_recalculate=false);
        void SetRollerPower(double power);
        bool HasPiece();
        bool ArmExtended();
        units::turn_t GetIntakePosition();
        units::revolutions_per_minute_t GetEncoderVelocity();
        bool AtSetPosition();
        void OpenLoopTestMotors(double pivot_power, double drive_power);

        //amp sutff
        void AmpMovement(double extend_power);

    private:
        bool _arm_sensor_hit = false;
        
        rev::CANSparkMax _pivot_motor;
        rev::CANSparkMax _drive_motor;

        frc::DigitalInput _piece_sensor;
        frc::DigitalInput _arm_sensor;

        rev::SparkRelativeEncoder* _pivot_encoder;
        rev::SparkPIDController* _pivot_pid_controller;

        frc::TrapezoidProfile<units::degrees> _intake_trapezoid{{IntakeConstants::MAX_VELOCITY, IntakeConstants::MAX_ACCELERATION}};
        frc::TrapezoidProfile<units::degree>::State _current_state{
            0_deg, 
            0_deg_per_s
        };
        frc::TrapezoidProfile<units::degree>::State _target_state{
            0_deg, 
            0_deg_per_s
        };
        units::degree_t _target_position;

        frc::Timer _trapezoid_timer;

        // Amp Stuff
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX _amp_motor;
};


#endif