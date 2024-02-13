// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/teleop/TeleopLauncherCommand.h"
#include "frc/smartdashboard/SmartDashboard.h"


using namespace LauncherConstants;
using namespace IntakeConstants;
using namespace VisionConstants;
using namespace frc;

TeleopLauncherCommand::TeleopLauncherCommand(LauncherSubsystem* launcher_subsystem, IntakeSubsystem* intake_subsystem, Vision* vision, Operator_Interface* oi)
: _launcher{launcher_subsystem},_intake{intake_subsystem}, _limelight{vision}, _oi{oi} { 
    AddRequirements(_launcher), AddRequirements(_intake);
}


void TeleopLauncherCommand::Initialize(){

    if(_intake !=NULL){
        _intake->SetIntakeAngle(STOW_POSITION);
        _intake->SetRollerPower(ROLLER_STOP);
        
    }

    _launching = false;
    if (_launcher !=NULL){
    _launcher->setLauncherRPM(TARGET_RPM);
    }


}
void TeleopLauncherCommand::Execute(){
    if (_launcher !=NULL && _intake != NULL){
        if(_launcher->atTargetRPM() && _intake->AtSetPosition() && ( _limelight == NULL || (_oi != NULL && _oi->IgnoreVision()) || (_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance()) < AIM_TOLERANCE_SMALL) )){
            _launching = true;
        }
        if(_launching){
            _intake->SetRollerPower(-ROLLER_POWER);
        }
        #ifdef EN_DIAGNOSTICS
            SmartDashboard::PutBoolean("Launcher: At Target RPM", _launcher->atTargetRPM());
            SmartDashboard::PutBoolean("Launcher: At Set Position", _intake->AtSetPosition());
            SmartDashboard::PutBoolean("Launcher: Has Target", _limelight->HasTarget());
            SmartDashboard::PutNumber("Launcher: Horizontal Distance", double(_limelight->GetHorizontalDistance().value()));
        #endif
    }

}
void TeleopLauncherCommand::End(bool interrupted){
    if (_launcher !=NULL && _intake != NULL){
        _launcher->setLauncherRPM(0_rpm);
        _intake->SetRollerPower(ROLLER_STOP);
    }
}
bool TeleopLauncherCommand::IsFinished(){
 return false;
}