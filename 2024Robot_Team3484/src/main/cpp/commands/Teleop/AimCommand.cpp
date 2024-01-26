#include "commands/Teleop/AimCommand.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

using namespace VisionConstants;

using namespace SwerveConstants::AutonDriveConstants;
using namespace SwerveConstants::BrakeConstants;


AimCommand::AimCommand(DrivetrainSubsystem* drivetrain, Driver_Interface* oi_driver, Operator_Interface* oi_operator, Vision* vision)
    : _drivetrain{drivetrain},
    _oi_driver{oi_driver},
    _oi_operator{oi_operator},
    _limelight{vision} {
    AddRequirements(_drivetrain);
}


void AimCommand::Initialize() {
    _oi_driver->SetRumble(.2);
    // fmt::print("Testing");
    //  Two constants for AIM_TOLERANCE HIGH and LOW
    // natural default is to break
    _aiming = false;
    _initial_positions = _drivetrain->GetModulePositions();

    if (_limelight == NULL) {
        fmt::print("Limelight is Null");
    }
    _limelight->SetCameraAngle(CAMERA_ANGLE);
    _limelight->SetLensHeight(CAMERA_HEIGHT);
    _limelight->SetTargetHeight(TARGET_HEIGHT);
}
void AimCommand::Execute() {
    SmartDashboard::PutNumber("Horizontal Distance", _limelight->GetHorizontalDistance().value());
    SmartDashboard::PutNumber("Horizontal Angle", _limelight->GetOffsetX());
    if (_aiming){
        _drivetrain->Drive(0_mps,0_mps,_limelight->GetOffsetX()*STEER_GAIN*MAX_ROTATION_SPEED, true);
        if ((_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance()) < AIM_TOLERANCE_SMALL) ||!_limelight->HasTarget() || _oi_operator->IgnoreVision()){
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



void AimCommand::End(bool interrupted) {
    _drivetrain->StopMotors();
    _drivetrain->SetCoastMode();
    _oi_driver->SetRumble(0);
}

bool AimCommand::IsFinished() {return false;}