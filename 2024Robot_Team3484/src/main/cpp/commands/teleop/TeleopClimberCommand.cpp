#include "commands/teleop/TeleopClimberCommand.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ClimberConstants;

TeleopClimberCommand::TeleopClimberCommand(ClimberSubsystem* climber_subsystem, Operator_Interface* oi)
    : _climber_subsystem{climber_subsystem}, _oi{oi} {}

void TeleopClimberCommand::Initialize() {}

void TeleopClimberCommand::Execute() {
    if (_oi != NULL && _climber_subsystem != NULL) {
        if (frc::SmartDashboard::GetBoolean("testing",true)) {
            _climber_subsystem->SetClimberPower(0);
            
            if(_oi->ClimberHotKey() && frc::SmartDashboard::GetBoolean("testing",true)) {
                _climber_subsystem->OpenLoopTestMotors(_oi->OpenLoopControlLeft(), _oi->OpenLoopControlRight());
            }
        }
    }
    else {
        if (_oi->ClimbUp()) {
            _climber_subsystem->SetClimberPower(ClimberConstants::MOTOR_UP_SPEED);


        } else if (_oi->ClimbDown()) {
            _climber_subsystem->SetClimberPower(ClimberConstants::MOTOR_DOWN_SPEED);

        } else {
            _climber_subsystem->SetClimberPower(ClimberConstants::MOTOR_STOP);
        }


    }   
}

void TeleopClimberCommand::End(bool inturrupted) {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {}
    else {
        _climber_subsystem->SetClimberPower(ClimberConstants::MOTOR_STOP);
    }
}

bool TeleopClimberCommand::IsFinished() {return false;}
