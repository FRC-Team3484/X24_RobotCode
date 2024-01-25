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
    _intake->SetIntakeAngle(STOW_POSITION);
    if(_intake !=NULL){
        _intake->SetRollerPower(ROLLER_STOP);
        _Loaded =false;
    }

    _Launching = false;
    if (_Launcher !=NULL){
    _Launcher->setLauncherRPM(Target_RPM);
    }


}
void LauncherCommand::Execute(){
    if (_Launcher !=NULL){
        if(_Launcher->atTargetRPM() && _Loaded == true){

            }
        
        }
}
void LauncherCommand::End(bool interrupted){
    if (_Launcher !=NULL){
        _Launcher->setLauncherRPM(0_rpm);
    }
}
bool LauncherCommand::IsFinished(){
 return false;
}
