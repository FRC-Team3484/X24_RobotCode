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
    _initial_positions = _drivetrain->GetModulePositions();
    if (_limelight == NULL) {
        fmt::print("Limelight is Null");
    }
    else {
    _limelight->SetCameraAngle(CAMERA_ANGLE);
    _limelight->SetLensHeight(CAMERA_HEIGHT);
    _limelight->SetTargetHeight(TARGET_HEIGHT);
    }
}
void AutonAimCommand::Execute() {
    if (_limelight == NULL) {
        fmt::print("Limelight is Null");
    } else {
        SmartDashboard::PutNumber("Horizontal Distance", _limelight->GetHorizontalDistance().value());
        SmartDashboard::PutNumber("Horizontal Angle", _limelight->GetOffsetX());
        if (_aiming){
            _drivetrain->Drive(0_mps,0_mps,_limelight->GetOffsetX()*STEER_GAIN*MAX_ROTATION_SPEED, true);
            if ((_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance()) < AIM_TOLERANCE_SMALL) ||!_limelight->HasTarget()){
                _aiming = false;
                _initial_positions = _drivetrain->GetModulePositions();
    }
        }
        else{
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

            if (_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance())>AIM_TOLERANCE_LARGE) {
                _aiming = true;
            }
        }
    }
}



void AutonAimCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
    _drivetrain->SetCoastMode();
}

bool AutonAimCommand::IsFinished() {return false;}