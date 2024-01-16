// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef SHOOTERH
#define SHOOTERH

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
// SH is for shooter everywhere
class ShooterSubsystem : public frc2::SubsystemBase {
    public:
        ShooterSubsystem();
        void Periodic() override;
        void setSHRPM(units::revolutions_per_minute_t speed);
        bool atPoint();

    private:
        rev::CANSparkMax SH_Left_motor{ShooterConstants::MOTOR_LEFT_PORT, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax SH_Right_motor{ShooterConstants::MOTOR_RIGHT_PORT, rev::CANSparkMax::MotorType::kBrushless};

        rev::SparkRelativeEncoder SH_Encoder_Left = SH_Left_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 );
        rev::SparkRelativeEncoder SH_Encoder_Right = SH_Right_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 );

        rev::SparkPIDController SH_m_Left_pidController = SH_Left_motor.GetPIDController();
        rev::SparkPIDController SH_m_Right_pidController = SH_Right_motor.GetPIDController();

        double _speed;

};
#endif