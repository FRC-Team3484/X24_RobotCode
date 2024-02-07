#include "commands/auton/AutonIntakeCommand.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace IntakeConstants;

AutonIntakeCommand::AutonIntakeCommand(IntakeSubsystem* intake_subsystem)
    : _intake_subsystem{intake_subsystem} {
        AddRequirements(_intake_subsystem);
}

void AutonIntakeCommand::Initialize() {
    _intake_subsystem->SetIntakeAngle(INTAKE_POSITION);
    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER);
}

void AutonIntakeCommand::Execute() {
    #ifdef EN_DIAGNOSTICS
        frc::SmartDashboard::PutBoolean("Intake: Has Piece", _intake_subsystem->HasPiece());
        frc::SmartDashboard::PutBoolean("Intake: Arm Extended", _intake_subsystem->ArmExtended());
        frc::SmartDashboard::PutBoolean("Intake: At Set Position", _intake_subsystem->AtSetPosition());
    #endif
}

void AutonIntakeCommand::End(bool inturrupted) {
    _intake_subsystem->SetIntakeAngle(STOW_POSITION);
    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
}

bool AutonIntakeCommand::IsFinished() {
    return _intake_subsystem->HasPiece();
}