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
        void SetDriverRumble(double Rumble);


        // Operator Controllers

    private:
        frc::XboxController _driver_controller{SwerveConstants::ControllerConstants::Driver::DRIVER_CONTROLLER_PORT};
};


class Operator_Interface{
    public:
        bool DeployIntake();
        bool Launch();
        bool Climb();
        bool IgnoreSensor();
        bool ExtendIntakeButton();
        bool IgnoreVision();
        bool EjectIntakeButton();
        bool IntakeOverrideButton();
        bool IntakeThroughShooterButton();
        bool LaunchButton();
        void SetOperatorRumble(double Rumble);
    private:
        frc::XboxController _operator_controller{SwerveConstants::ControllerConstants::Operator_Controler::OPERATOR_CONTROLLER_PORT};

};
#endif
