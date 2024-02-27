// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/auton/AutonLauncherCommand.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace LauncherConstants;
using namespace IntakeConstants;
using namespace VisionConstants;
using namespace frc;

#ifdef EN_TESTINING


#else

AutonLauncherCommand::AutonLauncherCommand(LauncherSubsystem* launcher_subsystem, IntakeSubsystem* intake_subsystem, Vision* vision)
: _launcher{launcher_subsystem},_intake{intake_subsystem}, _limelight{vision}{
    AddRequirements(_launcher), AddRequirements(_intake);
}


void AutonLauncherCommand::Initialize(){

    _timer.Reset();
    _timer.Start();

    if(_intake !=NULL){
        _intake->SetIntakeAngle(STOW_POSITION);
        _intake->SetRollerPower(ROLLER_STOP);
    }

    _launching = 0;

    if (_launcher !=NULL) {
        _launcher->setLauncherRPM(TARGET_RPM);
    }
}
void AutonLauncherCommand::Execute(){
    if (_launcher !=NULL && _intake != NULL) {
        if (_launcher->atTargetRPM() 
            && _intake->AtSetPosition() 
            && ( _limelight == NULL 
                || (_limelight->HasTarget() 
                && units::math::abs(_limelight->GetHorizontalDistance()) < AIM_TOLERANCE_LARGE) )) {
            _launching = 1;
        }

        if (_launching > 0) {
            _intake->SetRollerPower(-ROLLER_POWER);
        }
        if (_launching == 1 && _launcher->LaunchingSensor()) {
            _launching = 2;
        }
        // #ifdef EN_DIAGNOSTICS
        //     SmartDashboard::PutBoolean("Launcher: At Target RPM", _launcher->atTargetRPM());
        //     SmartDashboard::PutBoolean("Launcher: At Set Position", _intake->AtSetPosition());
        //     SmartDashboard::PutBoolean("Launcher: Has April Tag", _limelight->HasTarget());
        //     SmartDashboard::PutNumber("Launcher: Horizontal Distance", double(_limelight->GetHorizontalDistance().value()));
        // #endif
    }

}
void  AutonLauncherCommand::End(bool interrupted){
    _timer.Stop();
    if (_launcher !=NULL && _intake != NULL){
        _launcher->setLauncherRPM(0_rpm);
        _intake->SetRollerPower(ROLLER_STOP);
    }
}
bool  AutonLauncherCommand::IsFinished(){
 return _timer.HasElapsed(TIMEOUT) || (_launching == 2 && !_launcher->LaunchingSensor());
}
#endif
