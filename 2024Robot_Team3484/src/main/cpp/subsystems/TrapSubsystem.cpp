#include "subsystems/TrapSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
// Its a Trap!!!!!!!!!!!!!!!!!!!!!!!!
using namespace frc;
using namespace TrapConstants;
TrapSubsystem::TrapSubsystem(
    int _extension_motor_can_id,
    int _roller_motor_can_id,
    SC::SC_PIDConstants pidc,
    double pid_output_range_max,
    double pid_output_range_min

    ) : 
        _extension_motor{_extension_motor_can_id, rev::CANSparkMax::MotorType::kBrushless},
        _roller_motor{_roller_motor_can_id, rev::CANSparkMax::MotorType::kBrushless}
    {

    _extension_motor.RestoreFactoryDefaults();
    _roller_motor.RestoreFactoryDefaults();

    _extension_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    _roller_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    _extension_motor.SetInverted(TrapConstants::MOTOR_INVERTED);
    _roller_motor.SetInverted(TrapConstants::MOTOR_INVERTED);

    _extension_encoder = new rev::SparkRelativeEncoder(_extension_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kQuadrature, 4096));
    _extension_pid_controller = new rev::SparkPIDController(_extension_motor.GetPIDController());
    _extension_pid_controller->SetFeedbackDevice(*_extension_encoder);
    _extension_encoder->SetPosition(0);
    _target_position = HOME_POSITION;
}

void TrapSubsystem::Periodic() {
    #ifdef EN_DIAGNOSTICS
        SmartDashboard::PutNumber("Trap Extension (in)", _extension_encoder->GetPosition() * GEAR_RATIO.value());
    #endif
    const frc::TrapezoidProfile<units::inch>::State current_state(
        GetExtension(),
        GetExtensionVelocity()
    );
    const frc::TrapezoidProfile<units::inch>::State target_state(
        _target_position,
        0_fps
    );
    const units::inch_t linear_position = _extension_trapezoid.Calculate(20_ms, current_state, target_state).position;
    _extension_pid_controller->SetReference(linear_position.value(), rev::CANSparkMax::ControlType::kPosition);
}
void TrapSubsystem::SetRollerPower(double power) {
    _roller_motor.Set(power);

}
void TrapSubsystem::SetPosition(units::inch_t position){
    _target_position = position;
}
bool TrapSubsystem::AtPosition(){
    return units::math::abs(GetExtension() - _target_position) < POSITION_TOLORANCE;
}

units::inch_t TrapSubsystem::GetExtension(){
    return _extension_encoder->GetPosition() * GEAR_RATIO;
}
units::feet_per_second_t TrapSubsystem::GetExtensionVelocity(){
    return _extension_encoder->GetVelocity() * GEAR_RATIO / 1.0_min;
}