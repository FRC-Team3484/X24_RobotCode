// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LauncherCommand.h"

using namespace LauncherConstants;

LauncherCommand::LauncherCommand(LauncherSubsystem* Launcher_subsytem) 
: _Launcher{Launcher_subsytem}{
    AddRequirements(_Launcher);
}
void LauncherCommand::Initialize(){
    _Launching = false;
    if (_Launcher !=NULL){
    _Launcher->setLauncherRPM(Target_RPM);
    }
}
void LauncherCommand::Execute(){
    
    if (_Launcher !=NULL){
        if(_Launcher){ //This logic needs more of systems to be done but the logic would be if the intake is ready, chnage Launcher to true then fire

    }
        else{
            if(_Launcher->atTargetRPM()){//when intake is done replace with if(_Launcher->atPoint() and _intake->finshed angle()(**replace filler names with real ones**))   
                _Launching = true;
            }
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
