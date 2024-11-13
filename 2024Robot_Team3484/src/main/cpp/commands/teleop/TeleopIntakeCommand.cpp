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
            if (_operator_oi->IntakeHotKey()) {
                _intake_subsystem->OpenLoopTestMotors(_operator_oi->OpenLoopControlLeft(), _operator_oi->OpenLoopControlRight());
                //_intake_subsystem->OpenLoopTransferMotor(_operator_oi->OpenLoopControlRight());
            }
        } else {        
            if (_operator_oi->ExtendIntake()) {
                if ((!_intake_subsystem->HasPiece() || _operator_oi->IgnoreSensor())) {
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::INTAKE_POSITION);
                    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER);
                    // _intake_subsystem->SetTransferPower(IntakeConstants::ROLLER_POWER);

                } else {
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
                    // _intake_subsystem->SetTransferPower(IntakeConstants::ROLLER_STOP);
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
                    _intake_subsystem->SetTransferPower(IntakeConstants::TRANSFER_SHOOTER_POWER);
                    _launcher_subsystem->setLauncherSpeed(LauncherConstants::INTAKE_SPEED);

                } else {
                    _launcher_subsystem->setLauncherSpeed(LauncherConstants::ZERO_SPEED);
                    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
                    _intake_subsystem->SetTransferPower(IntakeConstants::ROLLER_STOP);
                }
            // Check Logic with Transfer Motor Here
            } else if (_operator_oi->LauncherToggle()) {
                //_intake_subsystem->AmpMovement(_operator_oi->AmpStick());
                if (_operator_oi->LauncherTrap()) {
                    fmt::print("Laucher Trap EXECUTING");
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                    _launcher_subsystem->setLauncherSpeed(LauncherConstants::FULL_SPEED);
                    if (_intake_subsystem->AtSetPosition() && _launcher_subsystem->atTargetRPM()) {
                        _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER*-1);
                        _intake_subsystem->SetTransferPower(IntakeConstants::TRANSFER_POWER*-1);
                    }

                } else if (_operator_oi->LauncherAmp()) {
                    fmt::print("Laucher Amp EXECUTING");
                    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                    _launcher_subsystem->setLauncherSpeed(LauncherConstants::AMP_SPEED);
                    
                    if (_intake_subsystem->AtSetPosition() && _launcher_subsystem->atTargetRPM()) {
                        _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER*-1);
                        _intake_subsystem->SetTransferPower(IntakeConstants::TRANSFER_POWER*-1);
                    }
                }
            } else {
                _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                _intake_subsystem->SetRollerPower(0);
                _intake_subsystem->SetTransferPower(0);
                _intake_timer.Stop();
                _intake_timer.Reset();

                _operator_oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);
                _driver_oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);

                if (_operator_oi->LauncherSpeaker()) {
                    _launcher_subsystem->setLauncherSpeed(LauncherConstants::SPEAKER_SPEED);
                } else {
                    _launcher_subsystem->setLauncherSpeed(LauncherConstants::ZERO_SPEED);
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
        _intake_subsystem->SetTransferPower(IntakeConstants::ROLLER_STOP);

        _launcher_subsystem->setLauncherSpeed(LauncherConstants::ZERO_SPEED);

        _operator_oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);
        _driver_oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);
    }
}

bool TeleopIntakeCommand::IsFinished() {return false;}
