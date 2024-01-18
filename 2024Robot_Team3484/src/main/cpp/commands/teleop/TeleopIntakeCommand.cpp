#include "commands/teleop/TeleopIntakeCommand.h"
#include <Constants.h>

using namespace IntakeConstants;

TeleopIntakeCommand::TeleopIntakeCommand(IntakeSubsystem* intake_subsystem, OI*, oi)
    : _intake_subsystem{intake_subsystem}, _oi{oi} {
        AddRequirements(_intake_subsystem);
}

void TeleopIntakeCommand::Initialize() {}

void TeleopIntakeCommand::Execute() {
    if (_oi->__) { // TODO: Set button
        if (!_intake_subsystem->HasPiece() || _oi->__) { // TODO: Set button
            _intake_subsystem->SetIntakeAngle(IntakeConstants::INTAKE_POSITION);
            _intake_subsystem->SetRollerPower(1);

        } else {
            _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
            _intake_subsystem->SetRollerPower(0);
        }

        if (_intake_subsystem->HasPiece()) {
            // TODO: Rumble command

        }

    } else if (_oi->__) { // TODO: Spit out piece button
        _intake_subsystem->SetIntakeAngle(IntakeConstants::INTAKE_POSITION);

        if (_intake_subsystem->ArmExtended()) {
            _intake_subsystem->SetRollerPower(-1);

        }

    } else {
        _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
        _intake_subsystem->SetRollerPower(0);
    }
}

void TeleopIntakeCommand::End(bool inturrupted) {}

bool TeleopIntakeCommand::IsFinished() {return false;}