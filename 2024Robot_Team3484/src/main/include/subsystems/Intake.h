#ifndef INTAKE_H
#define INTAKE_H


#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>

#include <Constants.h>

class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem();
        void Periodic() override;
        void SetIntakeAngle(double angle);
        void SetRollerPower(double power);
        bool HasPiece();
        bool ArmExtended();

    private:
        rev::CANSparkMax _pivot_motor{IntakeConstants::PIVOT_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax _drive_motor{IntakeConstants::DRIVE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushed};

        frc::DigitalInput _piece_sensor{IntakeConstants::PIECE_SENSOR_PORT};
        frc::DigitalInput _arm_sensor{IntakeConstants::ARM_SENSOR_PORT};

        rev::SparkRelativeEncoder _pivot_encoder = _pivot_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096);
        rev::SparkPIDController _pivot_pid_controller = _pivot_motor.GetPIDController();
};


#endif