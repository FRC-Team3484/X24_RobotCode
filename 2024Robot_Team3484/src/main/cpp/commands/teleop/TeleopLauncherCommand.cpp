#include "commands/teleop/TeleopLauncherCommand.h"
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace LauncherConstants;
using namespace IntakeConstants;
using namespace VisionConstants;
using namespace SwerveConstants::ControllerConstants;
using namespace frc;

TeleopLauncherCommand::TeleopLauncherCommand(LauncherSubsystem* launcher_subsystem, IntakeSubsystem* intake_subsystem, Vision* vision, Operator_Interface* oi)
: _launcher{launcher_subsystem},_intake{intake_subsystem}, _limelight{vision}, _oi{oi} { 
    AddRequirements(_launcher), AddRequirements(_intake);
}

void TeleopLauncherCommand::Initialize() {
    if (frc::SmartDashboard::GetBoolean("Launcher Diagnostics",false)) {
        //SmartDashboard::PutNumber("Distance to Target", _limelight->GetDistanceFromTarget());
    }
    _launching = 0;

    if (_launcher != NULL) {
        if (frc::SmartDashboard::GetBoolean("testing",true)) {
            _launcher->OpenLoopTestMotors(0,0);
        } else {
            _launcher->setLauncherSpeed(SPEAKER_SPEED);
        }
    }
    
    if (_intake != NULL) {
        if (frc::SmartDashboard::GetBoolean("testing", true)) {
            _intake->SetRollerPower(0);
            _intake->OpenLoopTestMotors(0,0);
            _intake->SetTransferPower(0);
        } else {
            _intake->SetIntakeAngle(STOW_POSITION);
            _intake->SetRollerPower(ROLLER_STOP);
            _intake->SetTransferPower(TRANSFER_STOP);
        }
    }
}

void TeleopLauncherCommand::Execute() {
    if (_launcher !=NULL && _intake != NULL) {
        if (frc::SmartDashboard::GetBoolean("testing",true)) {
            if (_oi->LauncherHotKey() &&  frc::SmartDashboard::GetBoolean("testing",true)) {
                _launcher->OpenLoopTestMotors(_oi->OpenLoopControlLeft(), _oi->OpenLoopControlRight());
            }
        } else {
            if (_launcher->atTargetRPM() 
            && _intake->AtSetPosition() 
            && ( _limelight == NULL 
                || (_oi != NULL && _oi->IgnoreVision()) 
                || (_limelight->HasTarget() 
                    && units::math::abs(_limelight->GetHorizontalDistance()) < SPEAKER_AIM_TOLERANCE_LARGE) )) {
                _launching = 1;
            }

            if (_launching > 0) {
                _intake->SetRollerPower(-ROLLER_POWER);
                _intake->SetTransferPower(-TRANSFER_POWER);
            }

            if (_launching == 1 && _launcher->LaunchingSensor()) {
                _launching = 2;
            }

            if (_launching == 2 && !_launcher->LaunchingSensor()) {
                _oi->SetRumble(OPERATOR_RUMBLE_HIGH);
            }
        }
    }
}
void TeleopLauncherCommand::End(bool interrupted) {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {}
    else {
        if (_launcher !=NULL && _intake != NULL) {
            _launcher->setLauncherSpeed(ZERO_SPEED);
            _intake->SetRollerPower(ROLLER_STOP);
            _intake->SetTransferPower(TRANSFER_STOP);
            _oi->SetRumble(RUMBLE_STOP);
        }
    }
}

bool TeleopLauncherCommand::IsFinished() {return false;}
