#include "subsystems/LauncherSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <numbers>

using namespace LauncherConstants;
using namespace rev;
using namespace frc;

LauncherSubsystem::LauncherSubsystem(
        int bottom_motor_can_id,
        int top_motor_can_id,
        int launch_sensor_di_ch,
        SC::SC_PIDConstants bottom_pidc,
        SC::SC_PIDConstants top_pidc,
        double rpm_window
    ):
    _bottom_motor{bottom_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
    _top_motor{top_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
    _launched_sensor{launch_sensor_di_ch}
    {
        _rpm_window = rpm_window;

        _bottom_launcher_encoder = new SparkRelativeEncoder(_bottom_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
        _top_launcher_encoder = new SparkRelativeEncoder(_top_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));

        _bottom_launcher_pid_controller = new SparkPIDController(_bottom_motor.GetPIDController());

        _top_launcher_pid_controller = new SparkPIDController(_top_motor.GetPIDController());
        
    if (_bottom_launcher_pid_controller !=NULL) {    
        _bottom_launcher_pid_controller->SetFeedbackDevice(*_bottom_launcher_encoder);
    }

    if (_top_launcher_pid_controller !=NULL) {
        _top_launcher_pid_controller->SetFeedbackDevice(*_top_launcher_encoder);
    }

    _bottom_motor.RestoreFactoryDefaults();
    _top_motor.RestoreFactoryDefaults();
    
    _bottom_motor.SetInverted(BOTTOM_MOTOR_INVERTED);
    _top_motor.SetInverted(TOP_MOTOR_INVERTED);
    _bottom_motor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    _top_motor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);

    _bottom_motor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 200);
    _top_motor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 200);

    if (_bottom_launcher_pid_controller !=NULL) {
        _bottom_launcher_pid_controller->SetP(bottom_pidc.Kp);
        _bottom_launcher_pid_controller->SetI(bottom_pidc.Ki);
        _bottom_launcher_pid_controller->SetD(bottom_pidc.Kd);
        _bottom_launcher_pid_controller->SetIZone(0);
        _bottom_launcher_pid_controller->SetFF(bottom_pidc.Kf);
        _bottom_launcher_pid_controller->SetOutputRange(-1, 1);
    }

    if (_top_launcher_pid_controller !=NULL) {
        _top_launcher_pid_controller->SetP(top_pidc.Kp);
        _top_launcher_pid_controller->SetI(top_pidc.Ki);
        _top_launcher_pid_controller->SetD(top_pidc.Kd);
        _top_launcher_pid_controller->SetIZone(0);
        _top_launcher_pid_controller->SetFF(top_pidc.Kf);
        _top_launcher_pid_controller->SetOutputRange(-1, 1);
    }
}

void LauncherSubsystem::setLauncherSpeed(SC::SC_LauncherSpeed speed) {
    _target_speed = SC::SC_LauncherSpeed(speed.Top_Power, speed.Top_Speed*GEAR_RATIO, speed.Bottom_Power, speed.Bottom_Speed*GEAR_RATIO);
}

bool LauncherSubsystem::LaunchingSensor() {
    return !_launched_sensor.Get();
}

void LauncherSubsystem::Periodic() {
    if (frc::SmartDashboard::GetBoolean("Launcher Diagnostics",false)) {
        SmartDashboard::PutNumber("Motor Speed Bottom (RPM)", _bottom_launcher_encoder->GetVelocity()/GEAR_RATIO);
        SmartDashboard::PutNumber("Motor Speed top (RPM)", _top_launcher_encoder->GetVelocity()/GEAR_RATIO);
        SmartDashboard::PutBoolean("Launched Sensor", LaunchingSensor());
        SmartDashboard::PutBoolean("Launcher: At Target RPM", atTargetRPM());
    }

    if (frc::SmartDashboard::GetBoolean("testing",true)) {}
    else {
        //if (_dbnc_launch_window != NULL) {
            //_en_launch = _dbnc_launch_window->Calculate(_WithinRPMWindow());
        //}else {
            //_en_launch = _WithinRPMWindow();
        //}
        _counter_not_null_top = 0;
        _counter_not_null_bottom = 0;


        if (_bottom_launcher_pid_controller !=NULL) {
            if (_target_speed.Bottom_Power == 0 && _target_speed.Bottom_Speed == 0_rpm) {
                _bottom_launcher_pid_controller->SetReference(0, rev::CANSparkMax::ControlType::kDutyCycle);
                _bottom_launcher_pid_controller->SetIAccum(0);
            } else if (_target_speed.Bottom_Power != 0) {
                _bottom_launcher_pid_controller->SetReference(_target_speed.Bottom_Power, rev::CANSparkMax::ControlType::kDutyCycle); 

            } else {
                _bottom_launcher_pid_controller->SetReference(_target_speed.Bottom_Speed.value(), rev::CANSparkMax::ControlType::kVelocity);
            }
            _counter_not_null_bottom++;
        }

        if (_top_launcher_pid_controller !=NULL) {
            if (_target_speed.Top_Power == 0 && _target_speed.Top_Speed == 0_rpm) {
                _top_launcher_pid_controller->SetReference(0, rev::CANSparkMax::ControlType::kDutyCycle);
                _top_launcher_pid_controller->SetIAccum(0);
            } else if (_target_speed.Top_Power != 0) {
                _top_launcher_pid_controller->SetReference(_target_speed.Top_Power, rev::CANSparkMax::ControlType::kDutyCycle); 

            } else {
                _top_launcher_pid_controller->SetReference(_target_speed.Top_Speed.value(), rev::CANSparkMax::ControlType::kVelocity);
            }
            _counter_not_null_top++;
        }
    }
}

bool LauncherSubsystem::atTargetRPM() {
    bool bottom_at_speed = false;
    bool top_at_speed = false;

    if (_counter_not_null_bottom > 0) {
        if (_target_speed.Bottom_Power != 0) {
            bottom_at_speed = (_bottom_launcher_encoder->GetVelocity()-_target_speed.Bottom_Speed.value()) * (_target_speed.Bottom_Speed.value() >= 0 ? 1 : -1) > 0;
        } else if (_target_speed.Bottom_Speed.value() != 0) {
            bottom_at_speed = std::abs(_bottom_launcher_encoder->GetVelocity()-_target_speed.Bottom_Speed.value()) < _rpm_window;
        } else {
            bottom_at_speed = true;
        }
    } else {
            bottom_at_speed = true;
    }

    if (_counter_not_null_top > 0) {
        if (_target_speed.Top_Power != 0) {
            top_at_speed = (_top_launcher_encoder->GetVelocity()-_target_speed.Top_Speed.value()) * (_target_speed.Top_Speed.value() >= 0 ? 1 : -1) > 0;
        } else if (_target_speed.Top_Speed.value() != 0) {
            top_at_speed = std::abs(_top_launcher_encoder->GetVelocity()-_target_speed.Top_Speed.value()) < _rpm_window;
        } else {
            top_at_speed = true;
        }
    } else {
            top_at_speed = true;
    }

    return bottom_at_speed && top_at_speed;
}

void LauncherSubsystem::OpenLoopTestMotors(double power_left, double power_right) {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {
        _bottom_motor.Set(power_left);
        _top_motor.Set(power_right);
    }
}

