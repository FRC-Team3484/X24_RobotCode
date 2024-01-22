#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <units/angular_velocity.h>
namespace LauncherConstants {
    constexpr int Motor_Left_CanID2 = 0;
    constexpr int Motor_Right_CanID2 = 1;

    constexpr double P_Launcher = 0;
    constexpr double I_Launcher = 0;
    constexpr double D_Launcher = 0;
    constexpr double FF_Launcher = 0;
    constexpr double RPM_Window_Launcher = 50;

    //constexpr bool IsLoaded = true;
    constexpr bool MOTOR_INVERTED = true;


    constexpr units::revolutions_per_minute_t Target_RPM/*place holder*/ = 2000_rpm;
}
namespace IntakeConstants {}
namespace HookConstants {}








namespace SwerveConstants {}

#endif