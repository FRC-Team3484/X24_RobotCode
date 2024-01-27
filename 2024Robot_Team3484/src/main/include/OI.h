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
        bool DummyInput();
        bool ExtendIntakeButton();
        bool EjectIntakeButton();
        bool IntakeOverrideButton();
        bool IntakeThroughShooterButton();
        bool LaunchButton();

        // Operator Controllers

    private:
        frc::XboxController _driver_controller{SwerveConstants::ControllerConstants::Driver::DRIVER_CONTROLLER_PORT};
};

class Operator_Interface{
    public:
        bool DeployIntake();
        bool Launch();
        bool IgnoreVision();
        bool Climb();
        bool IgnoreSensor();
        bool IgnoreVison();
       
    private:
        frc::XboxController _perator_controller{SwerveConstants::ControllerConstants::OPERATOR_CONTROLLER_PORT};

};
#endif
