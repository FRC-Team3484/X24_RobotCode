
#include "subsystems/LauncherSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <numbers>

using namespace LauncherConstants;
using namespace rev;
using namespace frc;

LauncherSubsystem::LauncherSubsystem(
        int left_motor_can_id,
        int right_motor_can_id,
        int launch_sensor_di_ch,
        SC::SC_PIDConstants left_pidc,
        SC::SC_PIDConstants right_pidc,
        double rpm_window
    ):
    _left_motor{left_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
    _right_motor{right_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
    _launched_sensor{launch_sensor_di_ch}
    {
        _rpm_window = rpm_window;
      
        _left_launcher_encoder = new SparkRelativeEncoder(_left_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
        _right_launcher_encoder = new SparkRelativeEncoder(_right_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));

    
        _left_launcher_pid_controller = new SparkPIDController(_left_motor.GetPIDController());
    
        _right_launcher_pid_controller = new SparkPIDController(_right_motor.GetPIDController());
        
    if (_left_launcher_pid_controller !=NULL){    
        _left_launcher_pid_controller->SetFeedbackDevice(*_left_launcher_encoder);
    }

    if (_right_launcher_pid_controller !=NULL){
        _right_launcher_pid_controller->SetFeedbackDevice(*_right_launcher_encoder);
    }

    _left_motor.RestoreFactoryDefaults();
    _right_motor.RestoreFactoryDefaults();
    
    _left_motor.SetInverted(LEFT_MOTOR_INVERTED);
    _right_motor.SetInverted(!LEFT_MOTOR_INVERTED);

    _left_motor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 200);
    _right_motor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus5, 200);


    if (_left_launcher_pid_controller !=NULL){
        _left_launcher_pid_controller->SetP(left_pidc.Kp);
        _left_launcher_pid_controller->SetI(left_pidc.Ki);
        _left_launcher_pid_controller->SetD(left_pidc.Kd);
        _left_launcher_pid_controller->SetIZone(0);
        _left_launcher_pid_controller->SetFF(left_pidc.Kf);
        _left_launcher_pid_controller->SetOutputRange(-1, 1);
        }
    if (_right_launcher_pid_controller !=NULL){
        _right_launcher_pid_controller->SetP(right_pidc.Kp);
        _right_launcher_pid_controller->SetI(right_pidc.Ki);
        _right_launcher_pid_controller->SetD(right_pidc.Kd);
        _right_launcher_pid_controller->SetIZone(0);
        _right_launcher_pid_controller->SetFF(right_pidc.Kf);
        _right_launcher_pid_controller->SetOutputRange(-1, 1);
    }
}

void LauncherSubsystem::setLauncherRPM(units::revolutions_per_minute_t speed){
    _target_speed = speed.value()*GEAR_RATIO;
}
bool LauncherSubsystem::LaunchingSensor(){
    return !_launched_sensor.Get();
}

void LauncherSubsystem::Periodic() {
    if (frc::SmartDashboard::GetBoolean("Launcher Diagnostics",false)){
        SmartDashboard::PutNumber("Motor Speed Left (RPM)", _left_launcher_encoder->GetVelocity()/GEAR_RATIO);
        SmartDashboard::PutNumber("Motor Speed Right (RPM)", _right_launcher_encoder->GetVelocity()/GEAR_RATIO);
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
        _counter_not_null_right = 0;
        _counter_not_null_left = 0;


        if (_left_launcher_pid_controller !=NULL){
            if (_target_speed == 0) {
                _left_launcher_pid_controller->SetReference(0, rev::CANSparkMax::ControlType::kDutyCycle);
                _left_launcher_pid_controller->SetIAccum(0);
            } else {
                _left_launcher_pid_controller->SetReference(_target_speed, rev::CANSparkMax::ControlType::kVelocity);
            }
            _counter_not_null_left++;
        }
        if (_right_launcher_pid_controller !=NULL){
            if (_target_speed == 0) {
                _right_launcher_pid_controller->SetReference(0, rev::CANSparkMax::ControlType::kDutyCycle);
                _right_launcher_pid_controller->SetIAccum(0);
            } else {
                _right_launcher_pid_controller->SetReference(_target_speed, rev::CANSparkMax::ControlType::kVelocity);
            }    
            _counter_not_null_right++;
        }

    }

}

bool LauncherSubsystem::atTargetRPM() {
    if (_counter_not_null_left + _counter_not_null_right == 2){
        return std::abs(_left_launcher_encoder->GetVelocity()-_target_speed) < _rpm_window && std::abs(_right_launcher_encoder->GetVelocity()-_target_speed) < _rpm_window;
    }
    else if (_counter_not_null_left == 1) {
        return std::abs(_left_launcher_encoder->GetVelocity()-_target_speed) < _rpm_window;   
    }
    else if (_counter_not_null_right == 1) {
        return std::abs(_right_launcher_encoder->GetVelocity()-_target_speed) < _rpm_window;   
    }
    else {
        return false;
    }
}


//bool LauncherSubsystem::atTargetRPM(){
    //return _en_launch;

//}

void LauncherSubsystem::OpenLoopTestMotors(double power_left, double power_right) {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {
        _left_motor.Set(power_left);
        _right_motor.Set(power_right);
    }
}

