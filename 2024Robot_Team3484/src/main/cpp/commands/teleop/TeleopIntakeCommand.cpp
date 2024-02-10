#include "commands/teleop/TeleopIntakeCommand.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace IntakeConstants;

TeleopIntakeCommand::TeleopIntakeCommand(IntakeSubsystem* intake_subsystem, LauncherSubsystem* launcher_subsystem, Operator_Interface* oi)
    : _intake_subsystem{intake_subsystem}, _launcher_subsystem{launcher_subsystem}, _oi{oi} {
        AddRequirements(_intake_subsystem);
        AddRequirements(_launcher_subsystem);
}

void TeleopIntakeCommand::Initialize() {}

void TeleopIntakeCommand::Execute() {
    if (_oi != NULL && _intake_subsystem != NULL) {
#ifdef EN_TESTING
        _intake_subsystem->SetIntakeAngle(0_deg);
        _intake_subsystem->SetRollerPower(0);
        
        if(_oi->IntakeHotKey() && _is_open_loop) {
            _intake_subsystem->OpenLoopTestMotors(_oi->OpenLoopControlLeft(), _oi->OpenLoopControlRight());
        }
#else
    
        if (_oi->ExtendIntakeButton()) {
            if ((!_intake_subsystem->HasPiece() || _oi->IgnoreSensor())) {
                _intake_subsystem->SetIntakeAngle(IntakeConstants::INTAKE_POSITION);
                _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER);

            } else {
                _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
            }

            if (_intake_subsystem->HasPiece()) {
                _oi->SetRumble(SwerveConstants::ControllerConstants::OPERATOR_RUMBLE_LOW);

            }

        } else if (_oi->EjectIntakeButton()) {
            _intake_subsystem->SetIntakeAngle(IntakeConstants::EJECT_POSITION);

            if (_intake_subsystem->AtSetPosition()) {
                _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER * -1);
            }

        } else if (_oi->IntakeThroughShooterButton()) {
            if (!_intake_subsystem->HasPiece() || _oi->IgnoreSensor()) {
                _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
                _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_POWER * -1);
                _launcher_subsystem->setLauncherRPM(LauncherConstants::REVERSE_RPM);

            } else {
                _launcher_subsystem->setLauncherRPM(0_rpm);
            }

        } else {
            _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);
            _intake_subsystem->SetRollerPower(0);

            _oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);

            if (_oi->LaunchButton()) {
                _launcher_subsystem->setLauncherRPM(LauncherConstants::TARGET_RPM);

            } else {
                _launcher_subsystem->setLauncherRPM(0_rpm);

            }
        }
    

    #ifdef EN_DIAGNOSTICS
        frc::SmartDashboard::PutBoolean("Intake: Has Piece", _intake_subsystem->HasPiece());
        frc::SmartDashboard::PutBoolean("Intake: Arm Extended", _intake_subsystem->ArmExtended());
        frc::SmartDashboard::PutBoolean("Intake: At Set Position", _intake_subsystem->AtSetPosition());
    #endif

#endif
    }
}

void TeleopIntakeCommand::End(bool inturrupted) {
    #ifdef EN_TESTING
    #else
    _intake_subsystem->SetRollerPower(IntakeConstants::ROLLER_STOP);
    _intake_subsystem->SetIntakeAngle(IntakeConstants::STOW_POSITION);

    _launcher_subsystem->setLauncherRPM(0_rpm);

    _oi->SetRumble(SwerveConstants::ControllerConstants::RUMBLE_STOP);

    #endif
}

bool TeleopIntakeCommand::IsFinished() {return false;}