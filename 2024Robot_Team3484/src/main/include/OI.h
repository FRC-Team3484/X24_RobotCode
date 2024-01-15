#ifndef OI_H
#define OI_H


#include "Constants.h"
#include <frc/XboxController.h>

class OI {
    public:
        double GetThrottle();
        double GetStraife();
        double GetRotation();
        bool GetResetHeading();
        bool GetBrake();
        bool GetStraightenWheels();
        bool GetSetBrakeMode();
        bool GetDisableBrakeMode();

    private:
        frc::XboxController _driver_controller{SwerveConstants::ControllerConstants::DRIVER_CONTROLLER_PORT};
};


#endif
