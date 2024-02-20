#include "commands/teleop/TeleopTrapCommand.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <Constants.h>

using namespace TrapConstants;

TeleopTrapCommand::TeleopTrapCommand(TrapSubsystem* trap_subsystem, Operator_Interface* oi)
    : _trap_subsystem{trap_subsystem}, _oi{oi} {
        AddRequirements(_trap_subsystem);
}

void TeleopTrapCommand::Initialize() {}

void TeleopTrapCommand::Execute() {
    if (_oi != NULL) {
        if (frc::SmartDashboard::GetBoolean("testing",true)) {
            if(_oi->IntakeHotKey() ) {
                _trap_subsystem->OpenLoopTestMotors(_oi->OpenLoopControlLeft(), _oi->OpenLoopControlRight());
            }
        }

        else {
                if (_oi->EndgameToggle()&&_oi->ScoreTrap()){
                    _trap_subsystem->SetPosition(TRAP_POSITION);
                    if (_trap_subsystem->AtPosition()) {
                        _trap_subsystem->SetRollerPower(EJECT_POWER);
                    }
                } else if (_oi->EndgameToggle()&&_oi->AmpTrap()) {
                    _trap_subsystem->SetPosition(AMP_POSITION);
                    if (_trap_subsystem->AtPosition()) {
                        _trap_subsystem->SetRollerPower(EJECT_POWER);
                    } 
                } else if (_oi->EndgameToggle()&&_oi->IntakeTrap()){
                    _trap_subsystem->SetPosition(INTAKE_POSITION);
                    _trap_subsystem->SetRollerPower(INTAKE_POWER);
                } else {
                    _trap_subsystem->SetPosition(HOME_POSITION);
                    _trap_subsystem->SetRollerPower(0);
            }
        }
    }
}

void TeleopTrapCommand::End(bool interupted) {
    _trap_subsystem->SetPosition(HOME_POSITION);
    _trap_subsystem->SetRollerPower(0);
}

bool TeleopTrapCommand::IsFinished() {
    return false;
}
