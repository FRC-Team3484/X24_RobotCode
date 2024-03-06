#include "commands/teleop/TeleopIntakeCommand.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace IntakeConstants;

TeleopIntakeCommand::TeleopIntakeCommand(IntakeSubsystem* intake_subsystem, LauncherSubsystem* launcher_subsystem, ClimberSubsystem* climber_subsystem, Operator_Interface* operator_oi, Driver_Interface* driver_oi)
    : _intake_subsystem{intake_subsystem}, _launcher_subsystem{launcher_subsystem}, _climber_subsystem{climber_subsystem}, _operator_oi{operator_oi}, _driver_oi{driver_oi} {
        AddRequirements(_intake_subsystem);
        AddRequirements(_launcher_subsystem);
}

void TeleopIntakeCommand::Initialize() {
    _intake_subsystem->OpenLoopTestMotors(0,0);
    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
}

void TeleopIntakeCommand::Execute() {
    if (_operator_oi != NULL && _intake_subsystem != NULL) {
        if (frc::SmartDashboard::GetBoolean("testing",true)) {
            if(_operator_oi->IntakeHotKey()) {
                _intake_subsystem->OpenLoopTestMotors(_operator_oi->OpenLoopControlLeft(), _operator_oi->OpenLoopControlRight());
            }
        }
        else {

            // if (!_climber_subsystem->GetLeftSensor() || !_climber_subsystem->GetRightSensor()){
            //     _intake_subsystem->SetIntakeAngle(IntakeConstants::CLIMB_POSITION);
            // }else {
        
            if (_operator_oi->ExtendIntake()) {
                if ((!_intake_subsystem->HasPiece() || _operator_oi->IgnoreSensor())) {
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::INTAKE_POSITION);
                    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER);

            } else {
                _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
            }

            if (_intake_subsystem->HasPiece()) {
                _operator_oi->SetRumble(SwerveConstants::ControllerConstants::OPERATOR_RUMBLE_HIGH);
                _driver_oi->SetRumble(SwerveConstants::ControllerConstants::DRIVER_RUMBLE_HIGH);

            }

            } else if (_operator_oi->EjectIntake()) {
                _intake_subsystem->SetIntakeAngle(IntakeConstants::EJECT_POSITION);

                if (_intake_subsystem->AtSetPosition()) {
                    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER * -1);
                }
            } else if ((_operator_oi->IntakeThroughShooter() && !_operator_oi->LauncherToggle()) || (_operator_oi->LauncherIntake() && _operator_oi->LauncherToggle())) {
                if (!_intake_subsystem->HasPiece() || _operator_oi->IgnoreSensor()) {
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                    _intake_subsystem->SetRollerPower(IntakeConstants::INTAKE_SHOOTER_POWER);
                    _launcher_subsystem->setLauncherRPM(LauncherConstants::REVERSE_RPM);

                } else {
                    _launcher_subsystem->setLauncherRPM(0_rpm);
                    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
                }
            } else if (_operator_oi->LauncherToggle()) {
                //_intake_subsystem->AmpMovement(_operator_oi->AmpStick());
                if (_operator_oi->LauncherTrap()) {
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                    _launcher_subsystem->setLauncherRPM(LauncherConstants::TRAP_RPM);
                    if (_intake_subsystem->AtSetPosition() && _launcher_subsystem->atTargetRPM()) {
                        _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER*-1);
                    }
                    if (_intake_subsystem->HasPiece()) {
                        _operator_oi->SetRumble(SwerveConstants::ControllerConstants::OPERATOR_RUMBLE_HIGH);
                        _driver_oi->SetRumble(SwerveConstants::ControllerConstants::DRIVER_RUMBLE_HIGH);

                    }
                }

                if (_operator_oi->LauncherAmp()) {
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                    _launcher_subsystem->setLauncherRPM(LauncherConstants::AMP_RPM);
                    if (_intake_subsystem->AtSetPosition() && _launcher_subsystem->atTargetRPM()) {
                        _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER*-1);
                    }
                }
            
            } else if (_operator_oi->EjectIntake()) {
                _intake_subsystem->SetIntakeAngle(IntakeConstants::EJECT_POSITION);

                if (_intake_subsystem->AtSetPosition()) {
                    _intake_subsystem->SetRollerPower(IntakeConstants::EJECT_POWER);
                }

            } else if ((_operator_oi->IntakeThroughShooter() && !_operator_oi->LauncherToggle()) || (_operator_oi->LauncherIntake() && _operator_oi->LauncherToggle())) {
                if (!_intake_subsystem->HasPiece() || _operator_oi->IgnoreSensor()) {
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                    _intake_subsystem->SetRollerPower(IntakeConstants::INTAKE_SHOOTER_POWER);
                    _launcher_subsystem->setLauncherRPM(LauncherConstants::REVERSE_RPM);

                } else {
                    _launcher_subsystem->setLauncherRPM(0_rpm);
                    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
                }

            } else {
                _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                _intake_subsystem->SetRollerPower(0);

                _operator_oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);
                _driver_oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);

                if (_operator_oi->LauncherSpeaker()) {
                    _launcher_subsystem->setLauncherRPM(LauncherConstants::TARGET_RPM);
                } else {
                    _launcher_subsystem->setLauncherRPM(0_rpm);
                }
            }
        }
    }
}

void TeleopIntakeCommand::End(bool inturrupted) {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {}
    else {
        _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
        _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);

        _launcher_subsystem->setLauncherRPM(0_rpm);

        _operator_oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);
        _driver_oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);
    }
}

bool TeleopIntakeCommand::IsFinished() {return false;}
