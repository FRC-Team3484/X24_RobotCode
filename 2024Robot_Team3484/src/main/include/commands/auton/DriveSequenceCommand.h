#ifndef DRIVE_SEQUENCE_COMMAND_H
#define DRIVE_SEQUENCE_COMMAND_H

#include "subsystems/DrivetrainSubsystem.h"
#include "commands/auton/GoToPoseCommand.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

class DriveSequenceCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, DriveSequenceCommand> {
    public:
        DriveSequenceCommand(DrivetrainSubsystem* drivetrain);
    };

#endif