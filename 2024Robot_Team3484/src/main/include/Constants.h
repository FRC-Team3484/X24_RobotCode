#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace ShooterConstants {}
namespace IntakeConstants {}
namespace ClawConstants {
    constexpr int LEFT_MOTOR_CAN_ID = 50;
    constexpr int RIGHT_MOTOR_CAN_ID = 51;
    constexpr int LEFT_SENSOR_DI_CH= 1;
    constexpr int RIGHT_SENSOR_DI_CH= 1;

    constexpr bool MOTOR_INVERTED = false;
    constexpr int MOTOR_STOP = 0;
}

namespace SwerveConstants {}

#endif