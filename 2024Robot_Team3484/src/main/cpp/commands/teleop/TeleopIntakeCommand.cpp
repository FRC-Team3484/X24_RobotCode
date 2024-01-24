#include "commands/teleop/TeleopIntakeCommand.h"
#include <Constants.h>

using namespace IntakeConstants;

TeleopIntakeCommand::TeleopIntakeCommand(IntakeSubsystem* intake_subsystem, Driver_Interface* oi)
    : _intake_subsystem{intake_subsystem}, _oi{oi} {
        AddRequirements(_intake_subsystem);
}

void TeleopIntakeCommand::Initialize() {}

void TeleopIntakeCommand::Execute() {
    if (_oi->DummyInput()) { // TODO: Set button
        if (!_intake_subsystem->HasPiece() || _oi->DummyInput()) { // TODO: Set button
            _intake_subsystem->SetIntakeAngle(IntakeConstants::INTAKE_POSITION);
            _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER);

        } else {
            _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
            _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
        }

        if (_intake_subsystem->HasPiece()) {
            // TODO: Rumble command

        }

    } else if (_oi->DummyInput()) { // TODO: Spit out piece button
        _intake_subsystem->SetIntakeAngle(IntakeConstants::INTAKE_POSITION);

        if (_intake_subsystem->AtSetPosition()) {
            _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER * -1);

        }

    } else {
        _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
        _intake_subsystem->SetRollerPower(0);
    }
}

void TeleopIntakeCommand::End(bool inturrupted) {}

bool TeleopIntakeCommand::IsFinished() {return false;}