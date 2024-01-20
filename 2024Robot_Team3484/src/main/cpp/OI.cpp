#include "OI.h"


#include <frc/MathUtil.h>

using namespace SwerveConstants::ControllerConstants::Driver;


double OI::GetThrottle() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(THROTTLE), JOYSTICK_DEADBAND);}
double OI::GetStraife() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(STRAFE), JOYSTICK_DEADBAND);}
double OI::GetRotation() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(ROTATION), JOYSTICK_DEADBAND);}

bool OI::GetResetHeading() {return _driver_controller.GetRawButton(RESET_HEADING);}
bool OI::GetBrake() {return _driver_controller.GetRawButton(BRAKE);}
bool OI::GetStraightenWheels() {return _driver_controller.GetRawButton(STRAIGHTEN_WHEELS);}
bool OI::GetSetBrakeMode() {return _driver_controller.GetRawButtonPressed(BRAKE_MODE);}
bool OI::GetDisableBrakeMode() {return _driver_controller.GetRawButtonPressed(DISABLE_BRAKE_MODE);}
void OI::SetRumble(double Rumble) {
    _driver_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}



