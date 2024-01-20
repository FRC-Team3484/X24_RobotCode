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
    _Launcher->setLauncherRPM(Trget_RPM);
}
void LauncherCommand::Execute(){
    if(_Launcher){ //here is where we would have our run intake

    }
    else{
        if(_Launcher->atTargetRPM()){//when intake is done replace with if(_Launcher->atPoint() and _intake->finshed angle()(**replace filler names with real ones**))   
            _Launching = true;
        }
    }
}
void LauncherCommand::End(bool interrupted){
    _Launcher->setLauncherRPM(0_rpm);
}
bool LauncherCommand::IsFinished(){
 return false;
}
