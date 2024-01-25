// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LauncherCommand.h"
#include "commands/teleop/TeleopIntakeCommand.h"

using namespace LauncherConstants;
using namespace IntakeConstants;

LauncherCommand::LauncherCommand(LauncherSubsystem* Launcher_subsystem, IntakeSubsystem* intake_subsystem )
: _Launcher{Launcher_subsystem},_intake{intake_subsystem}{ 
    AddRequirements(_Launcher), AddRequirements(_intake);
}


void LauncherCommand::Initialize(){

    if(_intake !=NULL){
        _intake->SetIntakeAngle(STOW_POSITION);
        _intake->SetRollerPower(ROLLER_STOP);
        
    }

    _Launching = false;
    if (_Launcher !=NULL){
    _Launcher->setLauncherRPM(Target_RPM);
    }


}
void LauncherCommand::Execute(){
    if (_Launcher !=NULL && _intake != NULL){
        if(_Launcher->atTargetRPM() && _intake->AtSetPosition()){
            _Launching = true;
            }
        if(_Launching){
            _intake->SetRollerPower(-ROLLER_POWER);
        }
        }
}
void LauncherCommand::End(bool interrupted){
    if (_Launcher !=NULL && _intake != NULL){
        _Launcher->setLauncherRPM(0_rpm);
        _intake->SetRollerPower(ROLLER_STOP);
    }
}
bool LauncherCommand::IsFinished(){
 return false;
}
