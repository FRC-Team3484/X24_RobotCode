#include "commands/teleop/TeleopClimberCommand.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ClimberConstants;

TeleopClimberCommand::TeleopClimberCommand(ClimberSubsystem* Climber_subsystem, Operator_Interface* oi)
    : _climber_subsystem{_climber_subsystem}, _oi{oi} {
        AddRequirements(_climber_subsystem);
}

void TeleopClimberCommand::Initialize() {}

void TeleopClimberCommand::Execute() {
    if (_oi != NULL && _climber_subsystem != NULL) {
        if (frc::SmartDashboard::GetBoolean("testing",true)) {
            _climber_subsystem->SetClimberPower(0);
            
            if(_oi->IntakeHotKey() && frc::SmartDashboard::GetBoolean("testing",true)) {
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

        #ifdef EN_DIAGNOSTICS
            frc::SmartDashboard::PutBoolean("Climber: Left Sensor", _climber_subsystem->GetLeftSensor());
            frc::SmartDashboard::PutBoolean("Climber: Right Sensor", _climber_subsystem->GetRightSensor());
        #endif
    }   
}

void TeleopClimberCommand::End(bool inturrupted) {
    if (frc::SmartDashboard::GetBoolean("testing",true)) {}
    else {
        _climber_subsystem->SetClimberPower(ClimberConstants::MOTOR_STOP);
    }
}

bool TeleopClimberCommand::IsFinished() {return false;}