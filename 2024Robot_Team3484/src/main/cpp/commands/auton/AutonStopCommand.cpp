#include "commands/auton/AutonStopCommand.h"
#include <Constants.h>

using namespace frc;

AutonStopCommand::AutonStopCommand(DrivetrainSubsystem* drivetrain_subsystem)
    : _drivetrain_subsystem{drivetrain_subsystem} {
        AddRequirements(_drivetrain_subsystem);
}

void AutonStopCommand::Initialize() {
    _drivetrain_subsystem->StopMotors();
    auto alliance = DriverStation::GetAlliance();
    if (alliance) {
        if (alliance.value() == DriverStation::Alliance::kRed) {
            _drivetrain_subsystem->SetHeading(_drivetrain_subsystem->GetPose().Rotation().Degrees() + 180_deg);
        }
    }
}

void AutonStopCommand::Execute() {}

void AutonStopCommand::End(bool inturrupted) {}

bool AutonStopCommand::IsFinished() {
    return true;
}