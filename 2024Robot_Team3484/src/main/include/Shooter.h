// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef SHOOTERH
#define SHOOTERH

#include <Constants.h>

#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/CANSparkMax.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
// SH is for shooter everywhere
class SH {
    public:
        void setSHRPM(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rotation, bool open_loop = false);

    private:
        rev::CANSparkMax SH_Left_motor{MOTOR_LEFT_PORT, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANSparkMax SH_Right_motor{MOTOR_RIGHT_PORT, rev::CANSparkMax::MotorType::kBrushless};

        rev::SparkMaxRelativeEncoder SH_Encoder_Left = SH_Left_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 );
        rev::SparkMaxRelativeEncoder SH_Encoder_Right = SH_Right_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096 );

        rev::SparkPIDController SH_m_Left_pidController = SH_Left_motor.GetPIDController();
        rev::SparkPIDController SH_m_Right_pidController = SH_Right_motor.GetPIDController();

};
#endif