#include "commands/DeployIntakeCommand.h"

using namespace ShooterConstants;

DeployIntakeCommand::DeployIntakeCommand(IntakeSubsystem* intake_subsystem, OI*, oi)
    : _intake_subsystem{intake_subsystem}, _oi{oi} {
        AddRequirements(_intake_subsystem);
}

void DeployIntakeCommand::Initialize() {Stop();}

void DeployIntakeCommand::Execute() {
    // TODO: Logic for command
}

void DeployIntakeCommand::End(bool inturrupted) {Stop();}

bool DeployIntakeCommand::IsFinished() {return false;}

void DeployIntakeCommand::Stop() {} // TODO: Logic for when the command is stopped