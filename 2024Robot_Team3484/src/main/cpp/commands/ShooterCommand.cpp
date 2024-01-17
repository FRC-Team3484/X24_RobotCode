// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCommand.h"

using namespace ShooterConstants;

ShooterCommand::ShooterCommand(ShooterSubsystem* shooter_subsytem) 
: _shooter{shooter_subsytem}{
    AddRequirements(_shooter);
}
void ShooterCommand::Initialize(){
    _shooter->setSHRPM(RPM);
}
void ShooterCommand::Execute(){
 if(_shooter->atPoint()){//when intake is done replace with if(_shooter->atPoint() and _intake->finshed angle()(**replace filler names with real ones**))   
    //run intake
 }
 
}
void ShooterCommand::End(bool interrupted){
    _shooter->setSHRPM(0_rpm);
}
bool ShooterCommand::IsFinished(){

}
