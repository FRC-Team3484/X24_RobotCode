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
        bool LowSpeed();
        // bool GetStraightenWheels();
        bool GetSetBrakeMode();
        bool GetDisableBrakeMode();
        void SetRumble(double Rumble);


        // Operator Controllers

    private:
        frc::XboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};
};

class Operator_Interface{
    public:
        bool Launch();
        bool IgnoreVision();
        bool ClimbUp();
        bool ClimbDown();
        bool IgnoreSensor();
        bool ExtendIntakeButton();
        bool EjectIntakeButton();
        bool IntakeThroughShooterButton();
        bool LaunchButton();
        void SetRumble(double Rumble);
    private:
        frc::XboxController _operator_controller{UserInterface::Operator::OPERATOR_CONTROLLER_PORT};
};
#endif
