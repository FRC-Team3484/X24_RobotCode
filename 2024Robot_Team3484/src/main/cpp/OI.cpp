#include "OI.h"
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace UserInterface::Driver;
using namespace UserInterface::Operator;
using namespace UserInterface::Testing;


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
//joystick
double Operator_Interface::OpenLoopControlLeft() {return frc::SmartDashboard::GetBoolean("testing",true) ? frc::ApplyDeadband(_operator_controller.GetRawAxis(TESTING_OPEN_LOOP_LEFT), TESTING_DEADBAND) : 0;}
double Operator_Interface::OpenLoopControlRight() {return frc::SmartDashboard::GetBoolean("testing",true) ? frc::ApplyDeadband(_operator_controller.GetRawAxis(TESTING_OPEN_LOOP_RIGHT), TESTING_DEADBAND) : 0;}
// Hotkeys
bool Operator_Interface::LauncherHotKey(){return frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawButton(Hotkey::LAUNCHER_HK) : false;}
bool Operator_Interface::IntakeHotKey(){return frc::SmartDashboard::GetBoolean("testing",true) ?  _operator_controller.GetRawButton(Hotkey::INTAKE_HK) : false;}
double Operator_Interface::ClimberHotKey(){return frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawAxis(Hotkey::CLIMBER_HK) : 0;}

// //Testing Values for Launcher
// // bool Operator_Interface::LauncherLeftMotorTest(){return _operator_controller.GetRawButton(Launcher::LEFT_MOTOR);}
// // bool Operator_Interface::LauncherRightMotorTest(){return _operator_controller.GetRawButton(Launcher::RIGHT_MOTOR);}
// bool Operator_Interface::LauncherSensorTest(){frc::SmartDashboard::GetBoolean("testing",true) ?  _operator_controller.GetRawButtonPressed(Launcher::SHOT_SENSOR) : false;}
//
// //Testing Values for Intake
// bool Operator_Interface::IntakeAngleStowTest(){frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawButtonPressed(Intake::STOW_ANGLE) : false;}
// bool Operator_Interface::IntakeAngleReadyTest(){frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawButtonPressed(Intake::READY_ANGLE) : false;}
// bool Operator_Interface::IntakePowerForwardTest(){frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawButton(Intake::ROLL_FORWARD) : false;}
// bool Operator_Interface::IntakePowerBackwardTest(){frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawButton(Intake::ROLL_BACKWARD) : false;}
// double Operator_Interface::IntakeSensorTest(){frc::SmartDashboard::GetBoolean("testing",true) ? _operator_controller.GetRawButton(Intake::HAS_PIECE_SENSOR) : false;}
//

bool Operator_Interface::EjectIntake() {return _operator_controller.GetRawButton(EJECT);}
bool Operator_Interface::ExtendIntake() {return _operator_controller.GetRawButton(EXTEND);}
bool Operator_Interface::IgnoreVision() {return _operator_controller.GetRawButton(IGNORE);}
bool Operator_Interface::IgnoreSensor() {return _operator_controller.GetRawButton(IGNORE);}
bool Operator_Interface::IntakeThroughShooter() {return _operator_controller.GetRawButton(INTAKE_SHOOTER);}
// Starts launch and aim
bool Operator_Interface::Launch() {return _operator_controller.GetRawButton(AIM_START);}
// Trap Device
bool Operator_Interface::IntakeTrap() {return false;}
bool Operator_Interface::ScoreTrap() {return false;}
bool Operator_Interface::AmpTrap() {return false;}
// Mapped to D-Pad
bool Operator_Interface::ClimbUp() {return _operator_controller.GetPOV()==315 || _operator_controller.GetPOV()<=45;}
bool Operator_Interface::ClimbDown() {return _operator_controller.GetPOV()<=225 && _operator_controller.GetPOV()>=135;}

void Operator_Interface::SetRumble(double Rumble) {
    _operator_controller.SetRumble(frc::GenericHID::kBothRumble, Rumble);
}
