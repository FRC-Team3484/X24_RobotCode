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
        bool StartAim();
        bool GetResetHeading();
        bool GetBrake();
        bool GetBrakePressed();
        // bool GetStraightenWheels();
        bool GetSetBrakeMode();
        bool GetDisableBrakeMode();
        void SetRumble(double Rumble);


        // Operator Controllers

    private:
        frc::XboxController _driver_controller{SwerveConstants::ControllerConstants::Driver::DRIVER_CONTROLLER_PORT};
};

#include <frc/XboxController.h>

class Operator_Interface{
    public:
        bool Climb();
        bool IgnoreVision();
        bool ExtendIntakeButton();
        bool EjectIntakeButton();
        bool IntakeThroughShooterButton();
        bool LaunchButton();
        void SetRumble(double Rumble);
       
    private:
        frc::XboxController _operator_controller{LauncherConstants::OPERATOR_CONTROLLER_PORT};

};
#endif
