#include "OI.h"


#include <frc/MathUtil.h>

using namespace UserInterface::Driver;
using namespace UserInterface::Operator;


double Driver_Interface::GetThrottle() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(THROTTLE), JOYSTICK_DEADBAND);}
double Driver_Interface::GetStrafe() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(STRAFE), JOYSTICK_DEADBAND);}
double Driver_Interface::GetRotation() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(ROTATION), JOYSTICK_DEADBAND);}
bool Driver_Interface::GetResetHeading() {return _driver_controller.GetRawButton(RESET_HEADING);}
bool Driver_Interface::GetBrake() {return _driver_controller.GetRawButton(BRAKE);}
bool Driver_Interface::GetBrakePressed() {return _driver_controller.GetRawButtonPressed(BRAKE);}
// bool Driver_Interface::GetStraightenWheels() {return _driver_controller.GetRawButton(STRAIGHTEN_WHEELS);}
bool Driver_Interface::GetSetBrakeMode() {return _driver_controller.GetRawButtonPressed(BRAKE_MODE);}
bool Driver_Interface::GetDisableBrakeMode() {return _driver_controller.GetRawButtonPressed(DISABLE_BRAKE_MODE);}
bool Driver_Interface::LowSpeed() {return _driver_controller.GetRawAxis(LOW_SPEED) > 0.5;}
void Driver_Interface::SetRumble(double Rumble) {
    _driver_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}
bool Operator_Interface::ExtendIntakeButton() {return _operator_controller.GetRawButton(EXTEND);}
bool Operator_Interface::EjectIntakeButton() {return _operator_controller.GetRawButton(EJECT);}
bool Operator_Interface::IgnoreVision() {return _operator_controller.GetRawButton(IGNORE);}
bool Operator_Interface::IgnoreSensor() {return _operator_controller.GetRawButton(IGNORE);}
bool Operator_Interface::IntakeThroughShooterButton() {return _operator_controller.GetRawButton(INTAKE_SHOOTER);}
// Starts launch and aim
bool Operator_Interface::LaunchButton() {return _operator_controller.GetRawButton(AIM_START);}
// Mapped to D-Pad
bool Operator_Interface::ClimbUp() {return _operator_controller.GetPOV()==315 || _operator_controller.GetPOV()<=45;}
bool Operator_Interface::ClimbDown() {return _operator_controller.GetPOV()<=225 && _operator_controller.GetPOV()>=135;}

void Operator_Interface::SetRumble(double Rumble) {
    _operator_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}
