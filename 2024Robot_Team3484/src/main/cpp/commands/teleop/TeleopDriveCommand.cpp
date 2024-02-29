
#include "commands/teleop/TeleopDriveCommand.h"

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <frc/kinematics/SwerveModuleState.h>

using namespace units;
using namespace frc;
using namespace SwerveConstants::AutonDriveConstants;
using namespace SwerveConstants::BrakeConstants;
using namespace SwerveConstants::DrivetrainConstants::JoystickScaling;

TeleopDriveCommand::TeleopDriveCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi) 
    : _drivetrain{drivetrain}, _oi{oi} {
    AddRequirements(_drivetrain);
}

void TeleopDriveCommand::Initialize() {
    _drivetrain->SetBrakeMode();
    _encoder_saved = false;
    _brake_timer.Stop();

}

void TeleopDriveCommand::Execute() {
    // Repeated Data Grabbers
    wpi::array<SwerveModulePosition, 4> current_positions = _drivetrain->GetModulePositions();

    // Logic with buttons
    if (_oi != NULL) {
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
            if (_brake_timer.Get() == 0_s) {
                _brake_timer.Reset();
                _brake_timer.Start();
            }
            if (_brake_timer.HasElapsed(BRAKE_DELAY) && !_encoder_saved) {
                _encoder_saved = true;
            }
            _drivetrain->SetModuleStates(
                {
                SwerveModuleState{(_encoder_saved ? -(_initial_positions[FL].distance - current_positions[FL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED : 0_fps), 45_deg},
                SwerveModuleState{(_encoder_saved ? -(_initial_positions[FR].distance - current_positions[FR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED : 0_fps), -45_deg},
                SwerveModuleState{(_encoder_saved ? -(_initial_positions[BL].distance - current_positions[BL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED : 0_fps), -45_deg},
                SwerveModuleState{(_encoder_saved ? -(_initial_positions[BR].distance - current_positions[BR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED : 0_fps), 45_deg}
                },
                true,
                false
            );
            
        } else {
            _brake_timer.Stop();
            _brake_timer.Reset();

            // Logic for actual joystick movements
            meters_per_second_t x_speed = -_oi->GetThrottle() * MAX_LINEAR_SPEED;
            meters_per_second_t y_speed = -_oi->GetStrafe() * MAX_LINEAR_SPEED;
            radians_per_second_t rotation = -_oi->GetRotation() * MAX_ROTATION_SPEED;

            if (_oi->LowSpeed()) {
                x_speed *= LOW_SCALE;
                y_speed *= LOW_SCALE;
                rotation *= LOW_SCALE;
            }
            
            _drivetrain->Drive(x_speed, y_speed, rotation, true);
        }

    }
}

void TeleopDriveCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
    _brake_timer.Stop();
}

bool TeleopDriveCommand::IsFinished() {return false;}