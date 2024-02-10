// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef LAUNCHER_H
#define LAUNCHER_H

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

#include <units/angle.h>
#include <units/angular_velocity.h>

class LauncherSubsystem : public frc2::SubsystemBase {
    public:
        LauncherSubsystem(
            int left_motor_can_id, 
            int right_motor_can_id,
            SC::SC_PIDConstants pidc,
            double rpm_window
        );
        void Periodic() override;
        void setLauncherRPM(units::revolutions_per_minute_t speed);
        bool atTargetRPM();
        void OpenLoopTestMotors(double power_left, double power_right);


    private:
        rev::CANSparkMax _left_motor;
        rev::CANSparkMax _right_motor;

        rev::SparkRelativeEncoder* _left_launcher_encoder;
        rev::SparkRelativeEncoder* _right_launcher_encoder;
        rev::SparkPIDController* _left_launcher_pid_controller;
        rev::SparkPIDController* _right_launcher_pid_controller;

        double _target_speed;
        int _counter_not_null_right;
        int _counter_not_null_left;

        double _rpm_window;

};
#endif
