// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/auton/AutonLauncherCommand.h"


using namespace LauncherConstants;
using namespace IntakeConstants;
using namespace VisionConstants;

AutonLauncherCommand::AutonLauncherCommand(LauncherSubsystem* Launcher_subsystem, IntakeSubsystem* intake_subsystem, Vision* vision)
: _Launcher{Launcher_subsystem},_intake{intake_subsystem}, _limelight{vision}{ 
    AddRequirements(_Launcher), AddRequirements(_intake);
}


void AutonLauncherCommand::Initialize(){

    if(_intake !=NULL){
        _intake->SetIntakeAngle(STOW_POSITION);
        _intake->SetRollerPower(ROLLER_STOP);
        
    }

    _Launching = false;
    if (_Launcher !=NULL){
    _Launcher->setLauncherRPM(TARGET_RPM);
    }


}
void AutonLauncherCommand::Execute(){
    if (_Launcher !=NULL && _intake != NULL){
        if(_Launcher->atTargetRPM() && _intake->AtSetPosition() && ( _limelight == NULL || (_limelight->HasTarget() && units::math::abs(_limelight->GetHorizontalDistance()) < AIM_TOLERANCE_SMALL) )){
            _Launching = true;
        }
        if(_Launching){
            _intake->SetRollerPower(-ROLLER_POWER);
        }
    }

}
void  AutonLauncherCommand::End(bool interrupted){
    if (_Launcher !=NULL && _intake != NULL){
        _Launcher->setLauncherRPM(0_rpm);
        _intake->SetRollerPower(ROLLER_STOP);
    }
}
bool  AutonLauncherCommand::IsFinished(){
 return false;
}
