#include <subsystems/IntakeSubsystem.h>
#include <units/angle.h>
#include <FRC3484_Lib/utils/SC_Datatypes.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace IntakeConstants;
using namespace frc;
using namespace ctre::phoenix;

IntakeSubsystem::IntakeSubsystem( // Reference constants in Robot.h in the intializer list
    int pivot_motor_can_id, 
    int drive_motor_can_id, 
    int piece_sensor_di_ch,
    int transfer_motor_id,
    int arm_sensor_di_ch,
    SC::SC_PIDConstants pivot_pidc,
    double pid_output_range_max,
    double pid_output_range_min,
    int amp_motor_id
    ) :
        _pivot_motor{pivot_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
        _drive_motor{drive_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
        _transfer_motor{transfer_motor_id},
        _piece_sensor{piece_sensor_di_ch},
        _arm_sensor{arm_sensor_di_ch},
        _amp_motor{amp_motor_id}
    {

    _pivot_encoder = new rev::SparkRelativeEncoder(_pivot_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    _pivot_pid_controller = new rev::SparkPIDController(_pivot_motor.GetPIDController());
    
    // Amp Stuff
    _amp_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255);
    _amp_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255);
    _amp_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255);
    _amp_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 200);
    _amp_motor.ConfigSupplyCurrentLimit(_amp_currrent_limit);
    _amp_motor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    _amp_motor.SetInverted(true);

    _transfer_motor.ConfigFactoryDefault();
    _transfer_motor.SetNeutralMode(motorcontrol::Brake);
    _transfer_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_1_General_, 255);
    _transfer_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_4_AinTempVbat_, 255);
    _transfer_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_12_Feedback1_, 255);
    _transfer_motor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrame::Status_14_Turn_PIDF1_, 200);
    _transfer_motor.SetInverted(!TRANSFER_MOTOR_INVERTED);

    _pivot_motor.RestoreFactoryDefaults();
    _drive_motor.RestoreFactoryDefaults();

    _pivot_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    //_pivot_motor.SetSmartCurrentLimit(PIVOT_STALL_LIMIT, PIVOT_FREE_LIMIT);

    _pivot_pid_controller->SetFeedbackDevice(*_pivot_encoder);

    _target_position = STOW_POSITION;

    _drive_motor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 200);
    _drive_motor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 200);
    _drive_motor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 200);
    _drive_motor.SetSmartCurrentLimit(DRIVE_STALL_LIMIT, DRIVE_FREE_LIMIT);

    _pivot_pid_controller->SetP(pivot_pidc.Kp);
    _pivot_pid_controller->SetI(pivot_pidc.Ki);
    _pivot_pid_controller->SetD(pivot_pidc.Kd);
    _pivot_pid_controller->SetIZone(PID_IZ_ZONE);
    _pivot_pid_controller->SetFF(pivot_pidc.Kf);
    _pivot_pid_controller->SetOutputRange(pid_output_range_min, pid_output_range_max);

    //_intake_dbnc = new Debouncer(INTAKE_DBNC_TIME, Debouncer::kRising);
    //_intake_dbnc->Calculate(_pivot_motor.GetOutputCurrent() > INTAKE_DBNC_THRESHOLD)
}

void IntakeSubsystem::Periodic() {
    if (frc::SmartDashboard::GetBoolean("Intake Diagnostics", false)) {
        SmartDashboard::PutNumber("Intake Angle (deg)", GetIntakePosition().value()*360);
        //SmartDashboard::PutNumber("Intake Velocity", _pivot_encoder->GetVelocity());
        SmartDashboard::PutBoolean("Arm Extened Sensor", ArmExtended());
        SmartDashboard::PutBoolean("Intake: At Set Position", AtSetPosition());
        SmartDashboard::PutNumber("Pivot Motor Current (amps)", _pivot_motor.GetOutputCurrent());
        SmartDashboard::PutNumber("Pivot Motor Temperature (celsius)", _pivot_motor.GetMotorTemperature());
        SmartDashboard::PutBoolean("Arm Sensor Hit Status", _arm_sensor_hit);
        SmartDashboard::PutNumber("Pivot Accumulator Value", _pivot_pid_controller->GetIAccum());
        SmartDashboard::PutNumber("Pivot Target Position", _target_position.value());
        SmartDashboard::PutBoolean("Has Note", HasPiece());
    }

    if (frc::SmartDashboard::GetBoolean("testing",true)) {}
    else {
        if (_arm_sensor_hit) {
            if (_target_position == STOW_POSITION && !ArmExtended()) {
                _pivot_pid_controller->SetReference(0, rev::CANSparkMax::ControlType::kDutyCycle);
                _pivot_pid_controller->SetIAccum(0);
                _pivot_encoder->SetPosition(IntakeConstants::STOW_POSITION.value());
            } else {
                units::turn_t linear_angle = _intake_trapezoid.Calculate(_trapezoid_timer.Get(), _current_state, _target_state).position;
                #ifdef EN_DIAGNOSTICS
                SmartDashboard::PutNumber(" Target Position (Trapezoid)", linear_angle.value()*360);
                #endif

                // if (units::math::abs(linear_angle - GetIntakePosition()) >= 40_deg){
                //     SetIntakeAngle(_target_position, true);
                // } else {
                _pivot_pid_controller->SetReference(linear_angle.value()*GEAR_RATIO, rev::CANSparkMax::ControlType::kPosition);
                // }
            }

        } else {
            _pivot_pid_controller->SetReference(HOME_POWER, rev::CANSparkMax::ControlType::kDutyCycle);
            if (!ArmExtended()) {
                _arm_sensor_hit = true;
                _pivot_encoder->SetPosition(IntakeConstants::STOW_POSITION.value());
            }
        }
    }
}

void IntakeSubsystem::SetIntakeAngle(units::degree_t angle, bool force_recalculate) {
    // Use the pivot motor and set the angle
    if (_arm_sensor_hit && (angle != _target_position || force_recalculate)) {
        _target_position = angle;
        _target_state.position = _target_position;
        _current_state.position = GetIntakePosition();
        _current_state.velocity = GetEncoderVelocity();
    
        _trapezoid_timer.Restart();
    }
}

void IntakeSubsystem::SetRollerPower(double power) {
    // Set the power level of the drive motor
    _drive_motor.Set(power);
}

void IntakeSubsystem::SetTransferPower(double power) {
    _transfer_motor.Set(motorcontrol::VictorSPXControlMode::PercentOutput, power);
}

bool IntakeSubsystem::HasPiece() {
    // Returns true is there's a game piece in the intake
    return !_piece_sensor.Get();
}

bool IntakeSubsystem::ArmExtended() {
    // Returns true if the intake arm is extended
    return _arm_sensor.Get();
}

units::turn_t IntakeSubsystem::GetIntakePosition() {
    // Returns the angle of the intake
    return units::turn_t{_pivot_encoder->GetPosition() / IntakeConstants::GEAR_RATIO};
}

units::revolutions_per_minute_t IntakeSubsystem::GetEncoderVelocity() {
    // Returns the velocity of the encoder
    return units::revolutions_per_minute_t{_pivot_encoder->GetVelocity()} / IntakeConstants::GEAR_RATIO;
}

bool IntakeSubsystem::AtSetPosition() {
    if (_arm_sensor_hit) {
        return units::math::abs(GetIntakePosition() - _target_position) < IntakeConstants::POSITION_TOLERANCE;
    } else {
        return false;
    }
}

void IntakeSubsystem::OpenLoopTestMotors(double pivot_power, double drive_power) {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {
        _pivot_motor.Set(pivot_power);
        _drive_motor.Set(drive_power);
    }
}

void IntakeSubsystem::OpenLoopTransferMotor(double tranfer_power) {
    _transfer_motor.Set(motorcontrol::VictorSPXControlMode::PercentOutput, tranfer_power);
}

//Amp
void IntakeSubsystem::AmpMovement(double extend_power) {
    _amp_motor.Set(extend_power);
}
