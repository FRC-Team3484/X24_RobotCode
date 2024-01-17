#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>

namespace ShooterConstants {}
namespace IntakeConstants {
    constexpr int PIVOT_MOTOR_PORT = 0;
    constexpr int DRIVE_MOTOR_PORT = 0;
    constexpr int PIECE_SENSOR_PORT = 0;
    constexpr int ARM_SENSOR_PORT = 0;

    constexpr units::degrees_per_second_t MAX_VELOCITY = 10_deg_per_s;
    constexpr units::degrees_per_second_squared_t MAX_ACCELERATION = 10_deg_per_s_sq;

    constexpr units::degree_t STOW_POSITION = 0_deg;
    constexpr units::degree_t INTAKE_POSITION = 180_deg;

    constexpr units::degrees_per_second_t HOME_VELOCITY = 5_deg_per_s;
}

namespace HookConstants {}

namespace SwerveConstants {}

#endif