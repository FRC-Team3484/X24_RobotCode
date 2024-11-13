#ifndef ROBOT_CONTAINER_H
#define ROBOT_CONTAINER_H

#include <frc2/command/CommandPtr.h>

class RobotContainer {
    public:
        RobotContainer();
        frc2::CommandPtr GetAutonomousCommand();

    private:
        void ConfigureBindings();
};

#endif