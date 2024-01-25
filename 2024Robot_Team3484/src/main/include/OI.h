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
        void SetDriverRumble(double Rumble);


        // Operator Controllers

    private:
        frc::XboxController _driver_controller{SwerveConstants::ControllerConstants::Driver::DRIVER_CONTROLLER_PORT};
};

#include <frc/XboxController.h>

class Operator_Interface{
    public:
        bool DeployIntake();
        bool Launch();
        bool Climb();
        bool IgnoreSensor();
        bool IgnoreVison();
        bool DummyInput();
        bool ExtendIntakeButton();
        bool EjectIntakeButton();
        bool IntakeOverrideButton();
        bool IntakeThroughShooterButton();
        bool LaunchButton();
        void SetOperatorRumble(double Rumble);
       
    private:
        frc::XboxController _Operator_controller{SwerveConstants::ControllerConstants::Operatiorr_Controler::Operatior_CONTROLLER_PORT};

};
#endif
