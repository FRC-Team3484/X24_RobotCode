#include "commands/SwerveTeleop/DynamicBrakeCommand.h"

#include <units/angle.h>

#include <frc/kinematics/SwerveModuleState.h>

using namespace frc;
using namespace SwerveConstants::BrakeConstants;
using namespace SwerveConstants::AutonDriveConstants;

DynamicBrakeCommand::DynamicBrakeCommand(DrivetrainSubsystem* drivetrain)
    : _drivetrain{drivetrain} {}

void DynamicBrakeCommand::Initialize() {
    _initial_positions = _drivetrain->GetModulePositions();
    _drivetrain->SetBrakeMode();
}

void DynamicBrakeCommand::Execute() {
    wpi::array<SwerveModulePosition, 4> current_positions = _drivetrain->GetModulePositions();

    _drivetrain->SetModuleStates(
        {
        SwerveModuleState{-(_initial_positions[FL].distance - current_positions[FL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, 45_deg},
        SwerveModuleState{-(_initial_positions[FR].distance - current_positions[FR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, -45_deg},
        SwerveModuleState{-(_initial_positions[BL].distance - current_positions[BL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, -45_deg},
        SwerveModuleState{-(_initial_positions[BR].distance - current_positions[BR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, 45_deg}
        },
        true,
        false
    );
}

void DynamicBrakeCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
    _drivetrain->SetCoastMode();
}

bool DynamicBrakeCommand::IsFinished() {return false;}