#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace ShooterConstants {}
namespace IntakeConstants {}
namespace ClawConstants {
    constexpr int LEFT_MOTOR_CAN_ID = 50;
    constexpr int RIGHT_MOTOR_CAN_ID = 51;
    constexpr int LEFT_SENSOR_DI_CH= 3;
    constexpr int RIGHT_SENSOR_DI_CH= 4;

    constexpr bool MOTOR_INVERTED = false;
    constexpr int MOTOR_STOP = 0;

    constexpr double MOTOR_UP_SPEED = 0.5;
    constexpr double MOTOR_DOWN_SPEED = -0.5;
}

namespace SwerveConstants {}

#endif