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
            int bottom_motor_can_id, 
            int top_motor_can_id,
            int launch_sensor_di_ch,
            SC::SC_PIDConstants _bottom_pidc,
            SC::SC_PIDConstants _top_pidc,
            double rpm_window
        );
        void Periodic() override;
        void setLauncherSpeed(SC::SC_LauncherSpeed speed);
        bool atTargetRPM();
        void OpenLoopTestMotors(double power_bottom, double power_top);
        bool LaunchingSensor();

    private:
        bool _WithinRPMWindow();
        rev::CANSparkMax _bottom_motor;
        rev::CANSparkMax _top_motor;

        rev::SparkRelativeEncoder* _bottom_launcher_encoder;
        rev::SparkRelativeEncoder* _top_launcher_encoder;
        rev::SparkPIDController* _bottom_launcher_pid_controller;
        rev::SparkPIDController* _top_launcher_pid_controller;
        //frc::Debouncer *_dbnc_launch_window; //avoid premature launch: debounce on rising edge (RE)
        bool _en_launch;
        frc::DigitalInput _launched_sensor;

        SC::SC_LauncherSpeed _target_speed;
        int _counter_not_null_top;
        int _counter_not_null_bottom;

        double _rpm_window;
};

#endif
