#include "OI.h"


#include <frc/MathUtil.h>

using namespace SwerveConstants::ControllerConstants::Driver;


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
bool Operator_Interface::ExtendIntakeButton() {return false;}
bool Operator_Interface::EjectIntakeButton() {return false;}
bool Operator_Interface::IgnoreVision() {return _operator_controller.GetRawButton(IGNORE_AIM);}
bool Operator_Interface::IgnoreSensor() {return false;}
bool Operator_Interface::IntakeThroughShooterButton() {return false;}
// Starts launch and aim
bool Operator_Interface::LaunchButton() {return _operator_controller.GetRawButton(AIM_START);}
bool Operator_Interface::ClimbUp() {return false;}
bool Operator_Interface::ClimbDown() {return false;}
void Operator_Interface::SetOperatorRumble(double Rumble) {
    _operator_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}
