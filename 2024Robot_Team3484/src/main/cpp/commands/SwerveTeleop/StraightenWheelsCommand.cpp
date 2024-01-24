#include "commands/SwerveTeleop/StraightenWheelsCommand.h"

using namespace frc;

StraightenWheelsCommand::StraightenWheelsCommand(DrivetrainSubsystem* drivetrain)
    : _drivetrain{drivetrain} {}

void StraightenWheelsCommand::Initialize() {}

void StraightenWheelsCommand::Execute() {

    _drivetrain->SetModuleStates(
        {
        SwerveModuleState{0_mps, 0_deg},
        SwerveModuleState{0_mps, 0_deg},
        SwerveModuleState{0_mps, 0_deg},
        SwerveModuleState{0_mps, 0_deg}
        },
        true,
        false
    );
}

void StraightenWheelsCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
}

bool StraightenWheelsCommand::IsFinished() {return false;}