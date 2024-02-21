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
        bool GetBrakePressed();

        bool GetSetBrakeMode();
        bool GetDisableBrakeMode();

        bool LowSpeed();
        void SetRumble(double Rumble);

        bool AimSequenceIgnore();


        // Operator Controllers

    private:
        frc::XboxController _driver_controller{UserInterface::Driver::DRIVER_CONTROLLER_PORT};
};

class Operator_Interface{
    public:
        //joystick
        double OpenLoopControlLeft();
        double OpenLoopControlRight();
        // Hotkeys
        bool LauncherHotKey();
        bool IntakeHotKey();
        bool TrapHotKey();
        bool ClimberHotKey();
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
        
  // None testing

        // Launcher
        bool IntakeThroughShooter();
        bool LauncherSpeaker();
        bool LauncherToggle();
        bool LauncherTrap();
        bool LauncherAmp();
        bool LauncherIntake();
        // Trap
        bool EndgameToggle();
        bool IntakeTrap();
        bool ScoreTrap();
        bool AmpTrap();
        // Climber
        bool ClimbUp();
        bool ClimbDown();
        // Intake
        bool EjectIntake();
        bool ExtendIntake();

        // Ignores
        bool IgnoreVision();
        bool IgnoreSensor();

        void SetRumble(double Rumble);

    private:
        frc::XboxController _operator_controller{UserInterface::Operator::OPERATOR_CONTROLLER_PORT};
};

#endif
