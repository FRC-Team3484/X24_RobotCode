#ifndef LAUNCHER_H
#define LAUNCHER_H

#include <Constants.h>

#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <frc/filter/Debouncer.h>

class LauncherSubsystem : public frc2::SubsystemBase {
    public:
        LauncherSubsystem(
            int left_motor_can_id, 
            int right_motor_can_id,
            int launch_sensor_di_ch,
            SC::SC_PIDConstants _left_pidc,
            SC::SC_PIDConstants _right_pidc,
            double rpm_window
        );
        void Periodic() override;
        void setLauncherRPM(units::revolutions_per_minute_t speed);
        bool atTargetRPM();
        void OpenLoopTestMotors(double power_left, double power_right);
        bool LaunchingSensor();

    private:
        bool _WithinRPMWindow();
        rev::CANSparkMax _left_motor;
        rev::CANSparkMax _right_motor;

        rev::SparkRelativeEncoder* _left_launcher_encoder;
        rev::SparkRelativeEncoder* _right_launcher_encoder;
        rev::SparkPIDController* _left_launcher_pid_controller;
        rev::SparkPIDController* _right_launcher_pid_controller;
        //frc::Debouncer *_dbnc_launch_window; //avoid premature launch: debounce on rising edge (RE)
        bool _en_launch;
        frc::DigitalInput _launched_sensor;

        double _target_speed;
        int _counter_not_null_right;
        int _counter_not_null_left;

        double _rpm_window;
};

#endif
