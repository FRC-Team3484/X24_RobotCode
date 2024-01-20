//
// Intake Subsystem
//
// -Noah, Aidan  & Ethan
//

#include <subsystems/IntakeSubsystem.h>
#include <units/angle.h>

using namespace IntakeConstants;

IntakeSubsystem::IntakeSubsystem(
    int PIVOT_MOTOR_CAN_ID, 
    int DRIVE_MOTOR_CAN_ID, 
    int PIECE_SENSOR_DI_CH, 
    int ARM_SENSOR_DI_CH,

    double PID_P,
    int PID_I,
    int PID_D,
    int PID_IZ_ZONE,
    int PID_FF,
    int PID_OUTPUTRANGE_MIN,
    int PID_OUTPUTRANGE_MAX

    ) :
        _pivot_motor{PIVOT_MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
        _drive_motor{DRIVE_MOTOR_CAN_ID, rev::CANSparkMax::MotorType::kBrushed},
        _piece_sensor{PIECE_SENSOR_DI_CH},
        _arm_sensor{ARM_SENSOR_DI_CH}
    {

    _pivot_encoder = new rev::SparkRelativeEncoder(_pivot_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096));
    _pivot_pid_controller = new rev::SparkPIDController(_pivot_motor.GetPIDController());
    
    _pivot_motor.RestoreFactoryDefaults();
    _drive_motor.RestoreFactoryDefaults();

    _drive_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    _pivot_pid_controller->SetFeedbackDevice(*_pivot_encoder);

    _target_position = STOW_POSITION;

    _pivot_pid_controller->SetP(PID_P);
    _pivot_pid_controller->SetI(PID_I);
    _pivot_pid_controller->SetD(PID_D);
    _pivot_pid_controller->SetIZone(PID_IZ_ZONE);
    _pivot_pid_controller->SetFF(PID_FF);
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

    const units::turn_t linear_angle = _intake_trapezoid.Calculate(20_ms, current_state, target_state).position;

    _pivot_pid_controller->SetReference(linear_angle.value(), rev::CANSparkMax::ControlType::kPosition);

    if (!ArmExtended() && !_arm_sensor_hit) {
        _arm_sensor_hit = true;
        _pivot_encoder->SetPosition(IntakeConstants::STOW_POSITION.value());
    }
}

void IntakeSubsystem::SetIntakeAngle(units::degree_t angle) {
    // Use the pivot motor and set the angle
    if (_arm_sensor_hit) _target_position = angle;
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
        return HOME_VELOCITY*20_ms;

    }
}

units::revolutions_per_minute_t IntakeSubsystem::GetEncoderVelocity() {
    // Returns the velocity of the encoder

    return units::revolutions_per_minute_t{_pivot_encoder->GetVelocity()} / IntakeConstants::GEAR_RATIO;
}