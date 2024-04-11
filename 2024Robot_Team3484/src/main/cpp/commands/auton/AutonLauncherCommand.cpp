#include "commands/auton/AutonLauncherCommand.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace LauncherConstants;
using namespace IntakeConstants;
using namespace VisionConstants;
using namespace frc;

AutonLauncherCommand::AutonLauncherCommand(LauncherSubsystem* launcher_subsystem, IntakeSubsystem* intake_subsystem, Vision* vision)
: _launcher{launcher_subsystem},_intake{intake_subsystem} ,_limelight{vision} {
    AddRequirements(_launcher), AddRequirements(_intake);
    _limelight = vision;
}

void AutonLauncherCommand::Initialize() {
    _timer.Reset();
    _timer.Start();

    if(_intake !=NULL){
        _intake->SetIntakeAngle(STOW_POSITION);
        _intake->SetRollerPower(ROLLER_STOP);
        _intake->SetTransferPower(TRANSFER_STOP);
    }

    _launching = 0;

    if (_launcher !=NULL) {
        _launcher->setLauncherRPM(TARGET_RPM);
    }
}

void AutonLauncherCommand::Execute() {
    if (_launcher !=NULL && _intake != NULL) {
        if (_launcher->atTargetRPM() 
            && _intake->AtSetPosition() 
            && _intake->HasPiece()
            && ( _limelight == NULL 
                || (_limelight->HasTarget() 
                && units::math::abs(_limelight->GetHorizontalDistance()) < SPEAKER_AIM_TOLERANCE_LARGE)
                || !_limelight->HasTarget())) {
            _launching = 1;
        }

        if (_launching > 0) {
            _intake->SetRollerPower(-ROLLER_POWER);
            _intake->SetTransferPower(-TRANSFER_POWER);
        }
        if (_launching == 1 && _launcher->LaunchingSensor()) {
            _launching = 2;
        }
    }
}

void AutonLauncherCommand::End(bool interrupted) {
    _timer.Stop();
    if (_launcher !=NULL && _intake != NULL) {
        _launcher->setLauncherRPM(0_rpm);
        _intake->SetRollerPower(ROLLER_STOP);
        _intake->SetTransferPower(TRANSFER_STOP);
    }
}
bool  AutonLauncherCommand::IsFinished(){
 return (_timer.HasElapsed(TIMEOUT) || (_launching == 2 && !_launcher->LaunchingSensor())) || (_launching == 0 && !_intake->HasPiece() && !_intake->ArmExtended());
}