
#include "commands/teleop/DriveCommand.h"

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <frc/kinematics/SwerveModuleState.h>

using namespace units;
using namespace frc;
using namespace SwerveConstants::AutonDriveConstants;
using namespace SwerveConstants::BrakeConstants;

DriveCommand::DriveCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi) 
    : _drivetrain{drivetrain}, _oi{oi} {
    AddRequirements(_drivetrain);
}

void DriveCommand::Initialize() {
        _drivetrain->SetBrakeMode();
}

void DriveCommand::Execute() {
    // Repeated Data Grabbers
    wpi::array<SwerveModulePosition, 4> current_positions = _drivetrain->GetModulePositions();

    // Logic with buttons
    if (_oi->GetBrakePressed()) {
        // Get Initial Values of the Wheels
        _initial_positions = _drivetrain->GetModulePositions();
    }
    if (_oi->GetResetHeading()) {
        _drivetrain->SetHeading();
    }

    if (_oi->GetSetBrakeMode()) {
        _drivetrain->SetBrakeMode();
    }
    if (_oi->GetDisableBrakeMode()) {
        _drivetrain->SetCoastMode();
    }
    if(_oi->GetBrake()) {
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
    // Logic for actual joystick movements
    const meters_per_second_t x_speed = -_oi->GetThrottle() * MAX_LINEAR_SPEED;
    const meters_per_second_t y_speed = -_oi->GetStrafe() * MAX_LINEAR_SPEED;
    const radians_per_second_t rotation = -_oi->GetRotation() * MAX_ROTATION_SPEED;
    
    _drivetrain->Drive(x_speed, y_speed, rotation, true);
}

void DriveCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
}

bool DriveCommand::IsFinished() {return false;}