#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <units/angular_velocity.h>
namespace ShooterConstants {
    constexpr int MOTOR_LEFT_PORT = 0;
    constexpr int MOTOR_RIGHT_PORT = 1;

    constexpr double P = 0;
    constexpr double I = 0;
    constexpr double D = 0;
    constexpr double FF = 0;
    constexpr double RPM_Window = 50;

    //constexpr bool IsLoaded = true;
    constexpr bool MOTER_LEFT_INVERTED = true;


    constexpr units::revolutions_per_minute_t RPM = 2000_rpm;
}
namespace IntakeConstants {}
namespace HookConstants {}








namespace SwerveConstants {}

#endif