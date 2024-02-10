#include "commands/teleop/TeleopTrapCommand.h"
#include <Constants.h>

using namespace TrapConstants;

TeleopTrapCommand::TeleopTrapCommand(TrapSubsystem* trap_subsystem, Operator_Interface* oi)
    : _trap_subsystem{trap_subsystem}, _oi{oi} {
        AddRequirements(_trap_subsystem);
}

void TeleopTrapCommand::Initialize() {}

void TeleopTrapCommand::Execute() {
    if (_oi != NULL) {
#ifdef EN_TESTING
    
#else
        if (_oi->ScoreTrap()){
            _trap_subsystem->SetPosition(TRAP_POSITION);
            if (_trap_subsystem->AtPosition()) {
                _trap_subsystem->SetRollerPower(EJECT_POWER);
            }
        } else if (_oi->ScoreAmp()) {
            _trap_subsystem->SetPosition(AMP_POSITION);
            if (_trap_subsystem->AtPosition()) {
                _trap_subsystem->SetRollerPower(EJECT_POWER);
            } 
        } else if (_oi->IntakeTrap()){
            _trap_subsystem->SetPosition(INTAKE_POSITION);
            _trap_subsystem->SetRollerPower(INTAKE_POWER);
        } else {
            _trap_subsystem->SetPosition(HOME_POSITION);
            _trap_subsystem->SetRollerPower(0);
        }
    
#endif
    }
}

void TeleopTrapCommand::End(bool interupted) {
    _trap_subsystem->SetPosition(HOME_POSITION);
    _trap_subsystem->SetRollerPower(0);
}

bool TeleopTrapCommand::IsFinished() {
    return false;
}