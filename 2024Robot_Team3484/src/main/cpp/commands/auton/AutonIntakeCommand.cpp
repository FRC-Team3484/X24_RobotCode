#include "commands/auton/AutonIntakeCommand.h"
#include <Constants.h>

using namespace IntakeConstants;

AutonIntakeCommand::AutonIntakeCommand(IntakeSubsystem* intake_subsystem)
    : _intake_subsystem{intake_subsystem} {
        AddRequirements(_intake_subsystem);
}

void AutonIntakeCommand::Initialize() {
    _intake_subsystem->SetIntakeAngle(INTAKE_POSITION);
    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER);
}

void AutonIntakeCommand::Execute() {}

void AutonIntakeCommand::End(bool inturrupted) {
    _intake_subsystem->SetIntakeAngle(STOW_POSITION);
    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
}

bool AutonIntakeCommand::IsFinished() {
    return _intake_subsystem->HasPiece();
}