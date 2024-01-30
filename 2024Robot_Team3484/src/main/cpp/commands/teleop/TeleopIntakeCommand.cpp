#include "commands/teleop/TeleopIntakeCommand.h"
#include <Constants.h>

using namespace IntakeConstants;

TeleopIntakeCommand::TeleopIntakeCommand(IntakeSubsystem* intake_subsystem, LauncherSubsystem* launcher_subsystem, Operator_Interface* oi)
    : _intake_subsystem{intake_subsystem}, _launcher_subsystem{launcher_subsystem}, _oi{oi} {
        AddRequirements(_intake_subsystem);
        AddRequirements(_launcher_subsystem);
}

void TeleopIntakeCommand::Initialize() {}

void TeleopIntakeCommand::Execute() {
    if (_oi->ExtendIntakeButton()) {
        if ((!_intake_subsystem->HasPiece() || _oi->IgnoreVision()) && _oi != NULL) {
            _intake_subsystem->SetIntakeAngle(IntakeConstants::INTAKE_POSITION);
            _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER);

        } else {
            _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
            _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
        }

        if (_intake_subsystem->HasPiece()) {
            _oi->SetRumble(SwerveConstants::ControllerConstants::OPERATOR_RUMBLE_LOW);

        }

    } else if (_oi->EjectIntakeButton()) {
        _intake_subsystem->SetIntakeAngle(IntakeConstants::EJECT_POSITION);

        if (_intake_subsystem->AtSetPosition()) {
            _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER * -1);
        }

    } else if (_oi->IntakeThroughShooterButton()) {
        
        
        if (!_intake_subsystem->HasPiece() || _oi->IgnoreVision()) {
            _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
            _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER * -1);
            _launcher_subsystem->setLauncherRPM(LauncherConstants::REVERSE_RPM);

        } else {
            _launcher_subsystem->setLauncherRPM(0_rpm);
        }

    } else {
        _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
        _intake_subsystem->SetRollerPower(0);

        _oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);

        if (_oi->LaunchButton()) {
            _launcher_subsystem->setLauncherRPM(LauncherConstants::TARGET_RPM);

        } else {
            _launcher_subsystem->setLauncherRPM(0_rpm);

        }
    }
}

void TeleopIntakeCommand::End(bool inturrupted) {
    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);

    _launcher_subsystem->setLauncherRPM(0_rpm);

    _oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);
}

bool TeleopIntakeCommand::IsFinished() {return false;}