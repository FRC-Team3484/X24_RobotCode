#include "commands/teleop/ClawCommand.h"
#include <Constants.h>

using namespace ClawConstants;

ClawCommand::ClawCommand(ClawSubsystem* claw_subsystem, Driver_Interface* oi)
    : _claw_subsystem{claw_subsystem}, _oi{oi} {
        AddRequirements(_claw_subsystem);
}

void ClawCommand::Initialize() {}

void ClawCommand::Execute() {
    if (_oi->DummyInput()) { // TODO: Set button
        _claw_subsystem->SetClawPower(ClawConstants::MOTOR_UP_SPEED);


    } else if (_oi->DummyInput()) { // TODO: Set button
        _claw_subsystem->SetClawPower(ClawConstants::MOTOR_DOWN_SPEED);

    } else {
        _claw_subsystem->SetClawPower(ClawConstants::MOTOR_STOP);

    }
}

void ClawCommand::End(bool inturrupted) {
    _claw_subsystem->SetClawPower(ClawConstants::MOTOR_STOP);

}

bool ClawCommand::IsFinished() {return false;}