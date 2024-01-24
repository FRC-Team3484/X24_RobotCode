#ifndef OI_H
#define OI_H


#include "Constants.h"
#include <frc/XboxController.h>

class Driver_Interface {
    
    public:
        //  Swerve Controllers
        double GetThrottle();
        double GetStrafe();
        double GetRotation();
        bool GetResetHeading();
        bool GetBrake();
        bool GetStraightenWheels();
        bool GetSetBrakeMode();
        bool GetDisableBrakeMode();
        void SetRumble(double Rumble);
        bool DummyInput();

        // Operator Controllers

    private:
        frc::XboxController _driver_controller{SwerveConstants::ControllerConstants::Driver::DRIVER_CONTROLLER_PORT};
};


#endif
