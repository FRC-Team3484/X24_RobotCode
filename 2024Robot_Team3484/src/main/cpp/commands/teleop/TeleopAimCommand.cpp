#include "commands/teleop/TeleopAimCommand.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
using namespace frc;

using namespace VisionConstants;
using namespace SwerveConstants::DrivetrainConstants::JoystickScaling;
using namespace SwerveConstants::AutonDriveConstants;
using namespace SwerveConstants::ControllerConstants;
using namespace SwerveConstants::BrakeConstants;
using namespace units;

TeleopAimCommand::TeleopAimCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi_driver, Operator_Interface* oi_operator, Vision* vision)
    : _drivetrain{drivetrain},
    _oi_driver{oi_driver},
    _oi_operator{oi_operator},
    _limelight{vision} {
    AddRequirements(_drivetrain);
}

void TeleopAimCommand::Initialize() {
    // natural default is to break
    _aiming = false;
    _initial_positions = _drivetrain->GetModulePositions();
    if (_limelight == NULL) {
        fmt::print("Limelight is Null");
    }
    else {
            _limelight->SetCameraAngle(CAMERA_ANGLE);
            _limelight->SetLensHeight(CAMERA_HEIGHT);
            _limelight->SetTargetHeight(SPEAKER_TARGET_HEIGHT);
    }
    _limelight->SetPipeline(0);
}

void TeleopAimCommand::Execute() {
    if(_oi_operator->LauncherToggle()){
        if (_limelight == NULL || _oi_driver->AimSequenceIgnore()) {
            meters_per_second_t x_speed = -_oi_driver->GetThrottle() * MAX_LINEAR_SPEED;
            meters_per_second_t y_speed = -_oi_driver->GetStrafe() * MAX_LINEAR_SPEED;
            radians_per_second_t rotation = -_oi_driver->GetRotation() * MAX_ROTATION_SPEED;

            if (_oi_driver->LowSpeed()) {
                x_speed *= LOW_SCALE;
                y_speed *= LOW_SCALE;
                rotation *= LOW_SCALE;
            }
            _oi_driver->SetRumble(RUMBLE_STOP);
        
            _drivetrain->Drive(x_speed, y_speed, rotation, true);
        }
        else {
            _oi_driver->SetRumble(DRIVER_RUMBLE_LOW);
            if (_aiming){
                _drivetrain->Drive((_limelight->GetDistanceFromTarget() - TRAP_TARGET_DISTANCE)*DISTANCE_GAIN*MAX_LINEAR_SPEED,
                                    0_mps,
                                    _limelight->GetOffsetX()*STEER_GAIN*MAX_ROTATION_SPEED, true);
                if ((_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance()) < TRAP_AIM_TOLERANCE_SMALL) ||!_limelight->HasTarget() || _oi_operator->IgnoreVision()) {
                    _aiming = false;
                    _initial_positions = _drivetrain->GetModulePositions();
                }
            }
            else {
                //wpi::array<SwerveModulePosition, 4> current_positions = _drivetrain->GetModulePositions();
                // _drivetrain->SetModuleStates(
                //     {
                //     SwerveModuleState{-(_initial_positions[FL].distance - current_positions[FL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, 45_deg},
                //     SwerveModuleState{-(_initial_positions[FR].distance - current_positions[FR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, -45_deg},
                //     SwerveModuleState{-(_initial_positions[BL].distance - current_positions[BL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, -45_deg},
                //     SwerveModuleState{-(_initial_positions[BR].distance - current_positions[BR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, 45_deg}
                //     },
                //     true,
                //     false
                // );

                if (_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance())>TRAP_AIM_TOLERANCE_LARGE) {
                    _aiming = true;
                }
            }
        }
    }
    else{
        if (_limelight == NULL || _oi_driver->AimSequenceIgnore()) {
            meters_per_second_t x_speed = -_oi_driver->GetThrottle() * MAX_LINEAR_SPEED;
            meters_per_second_t y_speed = -_oi_driver->GetStrafe() * MAX_LINEAR_SPEED;
            radians_per_second_t rotation = -_oi_driver->GetRotation() * MAX_ROTATION_SPEED;

            if (_oi_driver->LowSpeed()) {
                x_speed *= LOW_SCALE;
                y_speed *= LOW_SCALE;
                rotation *= LOW_SCALE;
            }
            _oi_driver->SetRumble(RUMBLE_STOP);
            
            _drivetrain->Drive(x_speed, y_speed, rotation, true);
        } 
        else {
            _oi_driver->SetRumble(DRIVER_RUMBLE_LOW);
            if (_aiming){
                _drivetrain->Drive(0_mps,0_mps,_limelight->GetOffsetX()*STEER_GAIN*MAX_ROTATION_SPEED, true);
                if ((_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance()) < SPEAKER_AIM_TOLERANCE_LARGE) ||!_limelight->HasTarget() || _oi_operator->IgnoreVision()) {
                    _aiming = false;
                    _initial_positions = _drivetrain->GetModulePositions();
                }
            }
            else {
                //wpi::array<SwerveModulePosition, 4> current_positions = _drivetrain->GetModulePositions();
                // _drivetrain->SetModuleStates(
                //     {
                //     SwerveModuleState{-(_initial_positions[FL].distance - current_positions[FL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, 45_deg},
                //     SwerveModuleState{-(_initial_positions[FR].distance - current_positions[FR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, -45_deg},
                //     SwerveModuleState{-(_initial_positions[BL].distance - current_positions[BL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, -45_deg},
                //     SwerveModuleState{-(_initial_positions[BR].distance - current_positions[BR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED, 45_deg}
                //     },
                //     true,
                //     false
                // );

                if (_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance())>SPEAKER_AIM_TOLERANCE_LARGE) {
                    _aiming = true;
                }
            }
        }
    }
    
}

void TeleopAimCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
    _drivetrain->SetCoastMode();
    _oi_driver->SetRumble(RUMBLE_STOP);
}

bool TeleopAimCommand::IsFinished() {return false;}
