//
// Intake Subsystem
//
// -Aidan & Noah

#include <subsystems/Intake.h>
#include <units/angle.h>

using namespace IntakeConstants;

IntakeSubsystem::IntakeSubsystem() {
    _pivot_motor.RestoreFactoryDefaults();
    _drive_motor.RestoreFactoryDefaults();

    _pivot_pid_controller.SetFeedbackDevice(_pivot_encoder);

    _target_position = STOW_POSITION;

    _pivot_pid_controller.SetP(0.1);
    _pivot_pid_controller.SetI(1e-4);
    _pivot_pid_controller.SetD(1);
    _pivot_pid_controller.SetIZone(0);
    _pivot_pid_controller.SetFF(0);
    _pivot_pid_controller.SetOutputRange(-1, 1);

}

void IntakeSubsystem::Periodic() {
    // Runs every 20ms

    const frc::TrapezoidProfile<units::degree>::State current_state{
        GetIntakePosition(), 
        units::revolutions_per_minute_t{_pivot_encoder.GetVelocity()}
    }; // units::turns converts the revolutions to a angle value

    const frc::TrapezoidProfile<units::degree>::State target_state{
        _target_position,
        0_deg_per_s
    };

    const units::turn_t linear_angle = _intake_trapezoid.Calculate(20_ms, current_state, target_state).position;

    _pivot_pid_controller.SetReference(linear_angle.value(), rev::CANSparkMax::ControlType::kPosition);

    if (ArmExtended() && !_arm_sensor_hit) {
        _arm_sensor_hit = true;
        _pivot_encoder.SetPosition(0);
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
        return units::turn_t{_pivot_encoder.GetPosition()};

    } else {
        return HOME_VELOCITY*20_ms;

    }

}