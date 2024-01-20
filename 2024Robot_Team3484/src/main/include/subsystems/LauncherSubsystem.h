// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef LAUNCHERH
#define LAUNCHERH

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include <units/angle.h>
#include <units/angular_velocity.h>

class LauncherSubsystem : public frc2::SubsystemBase {
    public:
        LauncherSubsystem(
            int MOTOR_LEFT_CAN_ID, 
            int MOTOR_RIGHT_CAN_ID,
            double P_Launcher,
            double I_Launcher,
            double D_Launcher,
            double FF_Launcher,
            double RPM_Window_Launcher 
        );
        void Periodic() override;
        void setLauncherRPM(units::revolutions_per_minute_t speed);
        bool atTargetRPM();

    private:
        rev::CANSparkMax _left_motor;
        rev::CANSparkMax _right_motor;

        rev::SparkRelativeEncoder* Launcher_Encoder_Left;
        rev::SparkRelativeEncoder* Launcher_Encoder_Right;
        rev::SparkPIDController* Launcher_m_Left_pidController;
        rev::SparkPIDController* Launcher_m_Right_pidController;

        double _target_speed;

};
#endif