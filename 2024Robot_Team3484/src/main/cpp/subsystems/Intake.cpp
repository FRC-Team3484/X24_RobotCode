#include <subsystems/Intake.h>

IntakeSubsystem::IntakeSubsystem() {
    _pivot_motor.RestoreFactoryDefaults();
    _drive_motor.RestoreFactoryDefaults();

    _pivot_pid_controller.SetFeedbackDevice(_pivot_encoder);

    _pivot_pid_controller.SetP(0.1);
    _pivot_pid_controller.SetI(1e-4);
    _pivot_pid_controller.SetD(1);
    _pivot_pid_controller.SetIZone(0);
    _pivot_pid_controller.SetFF(0);
    _pivot_pid_controller.SetOutputRange(-1, 1);

}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SetIntakeAngle(double angle) {
    // Use the pivot motor and set the angle

    
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