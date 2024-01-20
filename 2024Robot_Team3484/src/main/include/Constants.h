#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>

namespace ShooterConstants {}
namespace IntakeConstants {
    constexpr int PIVOT_MOTOR_CAN_ID = 0;
    constexpr int DRIVE_MOTOR_CAN_ID = 0;
    constexpr int PIECE_SENSOR_DI_CH = 0;
    constexpr int ARM_SENSOR_DI_CH = 0;
    constexpr double GEAR_RATIO = 62.5;

    constexpr units::degrees_per_second_t MAX_VELOCITY = 10_deg_per_s;
    constexpr units::degrees_per_second_squared_t MAX_ACCELERATION = 10_deg_per_s_sq;

    constexpr units::degree_t STOW_POSITION = 0_deg;
    constexpr units::degree_t INTAKE_POSITION = 180_deg;

    constexpr units::degrees_per_second_t HOME_VELOCITY = 5_deg_per_s;

    constexpr double PID_P = 0.1;
    constexpr int PID_I = 1e-4;
    constexpr int PID_D = 1;
    constexpr int PID_IZ_ZONE = 0;
    constexpr int PID_FF = 0;
    constexpr int PID_OUTPUTRANGE_MIN = -1;
    constexpr int PID_OUTPUTRANGE_MAX = 1;

    constexpr int ROLLER_POWER = 1;
    constexpr int ROLLER_REVERSE = -1;
    constexpr int ROLLER_STOP = 0;
}

namespace HookConstants {}

namespace SwerveConstants {}

#endif