#ifndef OI_H
#define OI_H


#include "Constants.h"

#include <frc/XboxController.h>

class Operator_Interface{
    public:
        bool DeployIntake();
        bool Launch();
        bool Climb();
        bool IgnoreSensor();
        bool IgnoreVison();
       
    private:
        //frc::XboxController _Operator_controller{SwerveConstants::ControllerConstants::OPERATOR_CONTROLLER_PORT};

};
#endif
