//
// Intake Subsystem
//
// -Noah, Aidan & Ethan
//

#include <subsystems/IntakeSubsystem.h>
#include <units/angle.h>
#include <FRC3484_Lib/utils/SC_Datatypes.h>

using namespace IntakeConstants;

IntakeSubsystem::IntakeSubsystem(
    int pivot_motor_can_id, 
    int drive_motor_can_id, 
    int piece_sensor_di_ch, 
    int arm_sensor_di_ch,
    SC::SC_PIDConstants PID_CONSTANTS,
    double PID_IZ_ZONE,
    double PID_OUTPUTRANGE_MIN,
    double PID_OUTPUTRANGE_MAX

    ) :
        _pivot_motor{pivot_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
        _drive_motor{drive_motor_can_id, rev::CANSparkMax::MotorType::kBrushed},
        _piece_sensor{piece_sensor_di_ch},
        _arm_sensor{arm_sensor_di_ch}
    {

    _pivot_encoder = new rev::SparkRelativeEncoder(_pivot_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096));
    _pivot_pid_controller = new rev::SparkPIDController(_pivot_motor.GetPIDController());
    
    _pivot_motor.RestoreFactoryDefaults();
    _drive_motor.RestoreFactoryDefaults();

    _drive_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    _pivot_pid_controller->SetFeedbackDevice(*_pivot_encoder);

    _target_position = STOW_POSITION;

    _pivot_pid_controller->SetP(PID_CONSTANTS.Kp);
    _pivot_pid_controller->SetI(PID_CONSTANTS.Ki);
    _pivot_pid_controller->SetD(PID_CONSTANTS.Kd);
    _pivot_pid_controller->SetIZone(PID_IZ_ZONE);
    _pivot_pid_controller->SetFF(PID_CONSTANTS.Kf);
    _pivot_pid_controller->SetOutputRange(PID_OUTPUTRANGE_MIN, PID_OUTPUTRANGE_MAX);
}

void IntakeSubsystem::Periodic() {
    // Runs every 20ms

    const frc::TrapezoidProfile<units::degree>::State current_state{
        GetIntakePosition(), 
        GetEncoderVelocity()
    }; // units::turns converts the revolutions to a angle value

    const frc::TrapezoidProfile<units::degree>::State target_state{
        _target_position,
        0_deg_per_s
    };

    if (_arm_sensor_hit) {
        const units::turn_t linear_angle = _intake_trapezoid.Calculate(20_ms, current_state, target_state).position;
        _pivot_pid_controller->SetReference(linear_angle.value(), rev::CANSparkMax::ControlType::kPosition);

    } else {
        _pivot_pid_controller->SetReference(IntakeConstants::HOME_POWER, rev::CANSparkMax::ControlType::kDutyCycle);

        if (!ArmExtended()) {
            _arm_sensor_hit = true;
            _pivot_encoder->SetPosition(IntakeConstants::STOW_POSITION.value());
        }
    }
}

void IntakeSubsystem::SetIntakeAngle(units::degree_t angle) {
    // Use the pivot motor and set the angle
    _target_position = angle;
}

void IntakeSubsystem::SetRollerPower(double power) {
    // Set the power level of the drive motor

    _drive_motor.Set(power);
}

bool IntakeSubsystem::HasPiece() {
    // Returns true is there's a game piece in the intake

    return _piece_sensor.Get();
}

bool IntakeSubsystem::ArmExtended() {
    // Returns true if the intake arm is extended

    return _arm_sensor.Get();
}

units::turn_t IntakeSubsystem::GetIntakePosition() {
    // Returns the angle of the intake

    if (_arm_sensor_hit) {
        return units::turn_t{_pivot_encoder->GetPosition() / IntakeConstants::GEAR_RATIO};

    } else {
        return 0_deg;

    }
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