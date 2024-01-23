#include "commands/SwerveTeleop/TeleopDriveCommand.h"

#include <units/velocity.h>
#include <units/angular_velocity.h>

using namespace units;
using namespace SwerveConstants::AutonDriveConstants;

TeleopDriveCommand::TeleopDriveCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi) 
    : _drivetrain{drivetrain}, _oi{oi} {
    AddRequirements(_drivetrain);
}

void TeleopDriveCommand::Initialize() {
        _drivetrain->SetBrakeMode();
}

void TeleopDriveCommand::Execute() {
    if (_oi->GetResetHeading()) {
        _drivetrain->SetHeading();
    }

    if (_oi->GetSetBrakeMode()) {
        _drivetrain->SetBrakeMode();
    }
    if (_oi->GetDisableBrakeMode()) {
        _drivetrain->SetCoastMode();
    }
    
    const meters_per_second_t x_speed = -_oi->GetThrottle() * MAX_LINEAR_SPEED;
    const meters_per_second_t y_speed = -_oi->GetStrafe() * MAX_LINEAR_SPEED;
    const radians_per_second_t rotation = -_oi->GetRotation() * MAX_ROTATION_SPEED;
    
    _drivetrain->Drive(x_speed, y_speed, rotation, true);
}

void TeleopDriveCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
}

bool TeleopDriveCommand::IsFinished() {return false;}