#include "commands/auton/AutonAimCommand.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace VisionConstants;
using namespace SwerveConstants::AutonDriveConstants;
using namespace SwerveConstants::BrakeConstants;


AutonAimCommand::AutonAimCommand(DrivetrainSubsystem* drivetrain, Vision* vision)
    : _drivetrain{drivetrain},
    _limelight{vision} {
    AddRequirements(_drivetrain);
}


void AutonAimCommand::Initialize() {
    _aiming = false;
    _encoder_saved = false;
    _brake_timer.Reset();
    _brake_timer.Start();
    _initial_positions = _drivetrain->GetModulePositions();
    if (_limelight == NULL) {
        fmt::print("Limelight is Null");
    }
    else {
        _limelight->SetCameraAngle(CAMERA_ANGLE);
        _limelight->SetLensHeight(CAMERA_HEIGHT);
        _limelight->SetTargetHeight(SPEAKER_TARGET_HEIGHT);
    }
}
void AutonAimCommand::Execute() {
    if (_limelight == NULL) {
        fmt::print("Limelight is Null");
    } else {
        if (_aiming){
            _drivetrain->Drive(0_mps,0_mps,_limelight->GetOffsetX()*STEER_GAIN*MAX_ROTATION_SPEED, true);
            if ((_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance()) < AIM_TOLERANCE_SMALL) ||!_limelight->HasTarget()){
                _aiming = false;
                _encoder_saved = false;
                _brake_timer.Reset();
                _brake_timer.Start();
                _initial_positions = _drivetrain->GetModulePositions();
            }
        }
        else{
            if (_brake_timer.HasElapsed(BRAKE_DELAY) && !_encoder_saved) {
                _current_positions = _drivetrain->GetModulePositions();
                _encoder_saved = true;
            }
            _drivetrain->SetModuleStates(
                {
                SwerveModuleState{(_encoder_saved ? -(_initial_positions[FL].distance - _current_positions[FL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED : 0_fps), 45_deg},
                SwerveModuleState{(_encoder_saved ? -(_initial_positions[FR].distance - _current_positions[FR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED: 0_fps), -45_deg},
                SwerveModuleState{(_encoder_saved ? -(_initial_positions[BL].distance - _current_positions[BL].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED : 0_fps), -45_deg},
                SwerveModuleState{(_encoder_saved ? -(_initial_positions[BR].distance - _current_positions[BR].distance) * DYNAMIC_BRAKE_SCALING * MAX_LINEAR_SPEED : 0_fps), 45_deg}
                },
                true,
                false
            );

            if (_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance())>AIM_TOLERANCE_LARGE) {
                _aiming = true;
            }
        }
        #ifdef EN_DIAGNOSTICS
            SmartDashboard::PutBoolean("Swerve: Drivetrain Aim Has April Tag", _limelight->HasTarget());
            SmartDashboard::PutNumber("Swerve: Horizontal Distance", _limelight->GetHorizontalDistance().value());
            SmartDashboard::PutNumber("Swerve: Horizontal Angle", _limelight->GetOffsetX());
        #endif
    }
}



void AutonAimCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
    _drivetrain->SetCoastMode();
    _brake_timer.Stop();
}

bool AutonAimCommand::IsFinished() {return false;}
