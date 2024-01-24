#include "OI.h"


#include <frc/MathUtil.h>

using namespace SwerveConstants::ControllerConstants::Driver;


double Driver_Interface::GetThrottle() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(THROTTLE), JOYSTICK_DEADBAND);}
double Driver_Interface::GetStrafe() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(STRAFE), JOYSTICK_DEADBAND);}
double Driver_Interface::GetRotation() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(ROTATION), JOYSTICK_DEADBAND);}

bool Driver_Interface::GetResetHeading() {return _driver_controller.GetRawButton(RESET_HEADING);}
bool Driver_Interface::GetBrake() {return _driver_controller.GetRawButton(BRAKE);}
bool Driver_Interface::GetStraightenWheels() {return _driver_controller.GetRawButton(STRAIGHTEN_WHEELS);}
bool Driver_Interface::GetSetBrakeMode() {return _driver_controller.GetRawButtonPressed(BRAKE_MODE);}
bool Driver_Interface::GetDisableBrakeMode() {return _driver_controller.GetRawButtonPressed(DISABLE_BRAKE_MODE);}
void Driver_Interface::SetRumble(double Rumble) {
    _driver_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}



