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
    } else {
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
            if ((_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance()) < SPEAKER_AIM_TOLERANCE_SMALL_AUTON) ||!_limelight->HasTarget()) {
                _aiming = false;
                _initial_positions = _drivetrain->GetModulePositions();
            }
        } else {
            wpi::array<frc::SwerveModulePosition, 4> _current_positions = _drivetrain->GetModulePositions();
            _drivetrain->SetModuleStates(
                {            
                    SwerveModuleState{0_mps, 45_deg},
                    SwerveModuleState{0_mps, -45_deg},
                    SwerveModuleState{0_mps, -45_deg},
                    SwerveModuleState{0_mps, 45_deg}
                },
                true,
                false
            );

            if (_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance())>SPEAKER_AIM_TOLERANCE_LARGE_AUTON) {
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
    _drivetrain->SetBrakeMode();
}

bool AutonAimCommand::IsFinished() {return false;}
