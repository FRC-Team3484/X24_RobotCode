#include "commands/teleop/TeleopClawCommand.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ClawConstants;

TeleopClawCommand::TeleopClawCommand(ClawSubsystem* claw_subsystem, Operator_Interface* oi)
    : _claw_subsystem{claw_subsystem}, _oi{oi} {
        AddRequirements(_claw_subsystem);
}

void TeleopClawCommand::Initialize() {}

void TeleopClawCommand::Execute() {
    if (_oi->ClimbUp()) {
        _claw_subsystem->SetClawPower(ClawConstants::MOTOR_UP_SPEED);


    } else if (_oi->ClimbDown()) {
        _claw_subsystem->SetClawPower(ClawConstants::MOTOR_DOWN_SPEED);

    } else {
        _claw_subsystem->SetClawPower(ClawConstants::MOTOR_STOP);
    }

    #ifdef EN_DIAGNOSTICS
        frc::SmartDashboard::PutBoolean("Claw: Left Sensor", _claw_subsystem->GetLeftSensor());
        frc::SmartDashboard::PutBoolean("Claw: Right Sensor", _claw_subsystem->GetRightSensor());
    #endif
}

void TeleopClawCommand::End(bool inturrupted) {
    _claw_subsystem->SetClawPower(ClawConstants::MOTOR_STOP);
}

bool TeleopClawCommand::IsFinished() {return false;}