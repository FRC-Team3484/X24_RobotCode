#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <units/angular_velocity.h>
namespace LauncherConstants {
    constexpr int MOTOR_LEFT_PORT2 = 0;
    constexpr int MOTOR_RIGHT_PORT2 = 1;

    constexpr double P_Launcher = 0;
    constexpr double I_Launcher = 0;
    constexpr double D_Launcher = 0;
    constexpr double FF_Launcher = 0;
    constexpr double RPM_Window_Launcher = 50;

    //constexpr bool IsLoaded = true;
    constexpr bool MOTER_INVERTED = true;


    constexpr units::revolutions_per_minute_t RPM/*place holder*/ = 2000_rpm;
}
namespace IntakeConstants {}
namespace HookConstants {}








namespace SwerveConstants {}

#endif