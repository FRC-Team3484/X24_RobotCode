#include "OI.h"
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace UserInterface::Driver;
using namespace UserInterface::Operator;
using namespace UserInterface::Testing;


// DRIVER
// Motion Drive
 Driver_Interface::Driver_Interface(){};
double Driver_Interface::GetThrottle() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(THROTTLE), DRIVER_JOYSTICK_DEADBAND);}
double Driver_Interface::GetStrafe() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(STRAFE), DRIVER_JOYSTICK_DEADBAND);}
double Driver_Interface::GetRotation() {return frc::ApplyDeadband(_driver_controller.GetRawAxis(ROTATION), DRIVER_JOYSTICK_DEADBAND);}
// Settings Drive
bool Driver_Interface::GetResetHeading() {return _driver_controller.GetRawButton(RESET_HEADING);}
bool Driver_Interface::GetBrake() {return _driver_controller.GetRawButton(BRAKE);}
bool Driver_Interface::GetBrakePressed() {return _driver_controller.GetRawButtonPressed(BRAKE);}
// bool Driver_Interface::GetStraightenWheels() {return _driver_controller.GetRawButton(STRAIGHTEN_WHEELS);}
bool Driver_Interface::GetSetBrakeMode() {return _driver_controller.GetRawButtonPressed(BRAKE_MODE);}
bool Driver_Interface::GetDisableBrakeMode() {return _driver_controller.GetRawButtonPressed(DISABLE_BRAKE_MODE);}
bool Driver_Interface::LowSpeed() {return _driver_controller.GetRawAxis(LOW_SPEED) > 0.5;}

bool Driver_Interface::AimSequenceIgnore() {return _driver_controller.GetRawButton(DRIVER_IGNORE);}
void Driver_Interface::SetRumble(double Rumble) {
    _driver_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}
// OPERATOR

// Intake
Operator_Interface::Operator_Interface(){};
bool Operator_Interface::EjectIntake() {return _operator_controller.GetRawButton(EJECT);}
bool Operator_Interface::ExtendIntake() {return _operator_controller.GetRawButton(EXTEND);}
bool Operator_Interface::IntakeThroughShooter() {return _operator_controller.GetRawButton(INTAKE_LAUNCHER);}

// Launcher
// Starts launch and aim
bool Operator_Interface::LauncherSpeaker() {return _operator_controller.GetRawButton(AIM_START);}
bool Operator_Interface::LauncherToggle() {return _operator_controller.GetRawAxis(LAUNCHER_TOGGLE_HK) > 0.5;}
bool Operator_Interface::LauncherAmp() {return _operator_controller.GetRawButton(LAUNCHER_AMP);}
bool Operator_Interface::LauncherTrap() {return _operator_controller.GetRawButton(LAUNCHER_TRAP);}
bool Operator_Interface::LauncherIntake() {return _operator_controller.GetRawButton(LAUNCHER_INTAKE);}



// Trap
bool Operator_Interface::EndgameToggle() {return _operator_controller.GetRawButton(ENDGAME_TOGGLE_HK);}
bool Operator_Interface::IntakeTrap() {return _operator_controller.GetRawButtonPressed(INTAKE_TRAP);} 
bool Operator_Interface::ScoreTrap() {return _operator_controller.GetRawButtonPressed(SCORE_TRAP);}
bool Operator_Interface::AmpTrap() {return _operator_controller.GetRawButtonPressed(AMP_TRAP);}

// Climb
// Mapped to D-Pad
bool Operator_Interface::ClimbUp() {return _operator_controller.GetPOV()==315 || _operator_controller.GetPOV()<=45;}
bool Operator_Interface::ClimbDown() {return _operator_controller.GetPOV()<=225 && _operator_controller.GetPOV()>=135;}

// Ignores
bool Operator_Interface::IgnoreVision() {return _operator_controller.GetRawButton(OPERATOR_IGNORE);}
bool Operator_Interface::IgnoreSensor() {return _operator_controller.GetRawButton(OPERATOR_IGNORE);}

void Operator_Interface::SetRumble(double Rumble) {
    _operator_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}

// TESTING
//joystick
double Operator_Interface::OpenLoopControlLeft() {return frc::SmartDashboard::GetBoolean("testing",true) ? frc::ApplyDeadband(_operator_controller.GetRawAxis(TESTING_OPEN_LOOP_LEFT)*frc::SmartDashboard::GetNumber("Joystick Value (Left)",0), TESTING_DEADBAND) : 0;}
double Operator_Interface::OpenLoopControlRight() {return frc::SmartDashboard::GetBoolean("testing",true) ? frc::ApplyDeadband(_operator_controller.GetRawAxis(TESTING_OPEN_LOOP_RIGHT)*frc::SmartDashboard::GetNumber("Joystick Value (Right)",0), TESTING_DEADBAND) : 0;}
// Hotkeys
bool Operator_Interface::LauncherHotKey(){return frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawButton(Hotkey::LAUNCHER_HK) : false;}
bool Operator_Interface::IntakeHotKey(){return frc::SmartDashboard::GetBoolean("testing",true) ?  _operator_controller.GetRawButton(Hotkey::INTAKE_HK) : false;}
bool Operator_Interface::TrapHotKey(){return frc::SmartDashboard::GetBoolean("testing",true) ?  _operator_controller.GetRawAxis(Hotkey::TRAP_HK)>.5 : false;}
bool Operator_Interface::ClimberHotKey(){return frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawAxis(Hotkey::CLIMBER_HK)>.5 : false;}
