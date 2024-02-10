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
#ifdef EN_TESTING
        //joystick
        double OpenLoopControlLeft();
        double OpenLoopControlRight();
        // Hotkeys
        bool LauncherHotKey();
        bool IntakeHotKey();
        double ClimberHotKey();
        // Launcher
        bool LauncherLeftMotorTest();
        bool LauncherRightMotorTest();
        bool LauncherSensorTest();
        
        // Intake
        bool IntakeAngleStowTest();
        bool IntakeAngleReadyTest();
        bool IntakePowerForwardTest();
        bool IntakePowerBackwardTest();
        double IntakeSensorTest();
//#else
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

#endif

    private:
        frc::XboxController _operator_controller{UserInterface::Operator::OPERATOR_CONTROLLER_PORT};
};

#endif
