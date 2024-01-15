#include "OI.h"

#include "OI.h"

#include <frc/MathUtil.h>

using namespace SwerveConstants::ControllerConstants;

double OI::GetThrottle() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(Throttle), JOYSTICK_DEADBAND);}
double OI::GetStraife() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(Straife), JOYSTICK_DEADBAND);}
double OI::GetRotation() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(Rotate), JOYSTICK_DEADBAND);}

bool OI::GetResetHeading() {return _driver_controller.GetRawButton(ResetHeading);}
bool OI::GetBrake() {return _driver_controller.GetRawButton(Brake);}
bool OI::GetStraightenWheels() {return _driver_controller.GetRawButton(StraightenWheels);}
bool OI::GetSetBrakeMode() {return _driver_controller.GetRawButtonPressed(BrakeMode);}
bool OI::GetDisableBrakeMode() {return _driver_controller.GetRawButtonPressed(DisableBrakeMode);}
