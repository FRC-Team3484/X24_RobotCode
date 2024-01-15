#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h> 

#include <ctre/Phoenix.h>

using namespace units;

namespace ShooterConstants {}
namespace IntakeConstants {}
namespace HookConstants {}








namespace SwerveConstants {
    namespace ControllerConstants {
    constexpr int DRIVER_CONTROLLER_PORT = 0;
    constexpr double JOYSTICK_DEADBAND = 0.02;
    }

    namespace DrivetrainConstants {
        #define FL 0
        #define FR 1
        #define BL 2
        #define BR 3

        constexpr int DRIVE_MOTOR_PORTS[] = {9, 1, 4, 5}; //9, 1, 4, 5
        constexpr int STEER_MOTOR_PORTS[] = {8, 3, 2, 0}; //8, 3, 2, 0
        constexpr int ENCODER_PORTS[] = {2, 3, 0, 1};

        constexpr bool STEER_MOTOR_REVERSED = false;
        constexpr bool ENCODER_REVERSED = false;


        const motorcontrol::supplycurrentlimitconfiguration DRIVE_CURRENT_LIMIT{true, 35, 60, 0.1};
        const motorcontrol::supplycurrentlimitconfiguration STEER_CURRENT_LIMIT{true, 25, 40, 0.1};

        constexpr double ENCODER_OFFSET[] = {4.394, 71.630, -26.103, -71.455};

        constexpr inch_t DRIVETRAIN_WIDTH = 24_in;
        constexpr inch_t DRIVETRAIN_LENGTH = 24_in;
        constexpr double DRIVE_GEAR_RATIO = 36000.0/5880.0;
        constexpr double STEER_GEAR_RATIO = 12.8;
        constexpr int DRIVE_TICKS_PER_REVOLUTION =2048;
        constexpr int STEER_TICKS_PER_REVOLUTION = 2048;
        constexpr inch_t WHEEL_RADIUS = 2_in;

        constexpr feet_per_second_t MAX_WHEEL_SPEED = 8_fps;
        constexpr feet_per_second_squared_t MAX_WHEEL_ACCELARATION = 4_fps_sq;

        namespace DrivePIDConstants {
            constexpr double P = 1.0;
            constexpr double I = 0.0;
            constexpr double D = 0.0;
        }
        namespace DriveFeedForwardConstants { //Future Check
            constexpr volt_t S = 1.0_V;
            constexpr auto V = 0.8_V / 1.0_mps;
            constexpr auto A = 0.15_V / 1.0_mps_sq;
        }
        namespace SteerPIDConstants {
            constexpr double P = 0.5;
            constexpr double I = 0.0;
            constexpr double D = 0.0;
            constexpr radians_per_second_t MAX_SPEED = 12_rad_per_s;
            constexpr radians_per_second_squared_t MAX_ACCELRATION  = 100_rad_per_s_sq;
        }
    }

    namespace BrakeConstants {
        constexpr auto DYNAMIC_BRAKE_SCALING = -00.2/1_in;
    }

    namespace AutonDriveConstants {
        // How fast the robot can move in autons
        constexpr feet_per_second_t MAX_LINEAR_SPEED = 8_fps;
        constexpr feet_per_second_squared_t MAX_LINEAR_ACCELARATION = 4_fps_sq;
        constexpr radians_per_second_t MAX_ROTATION_SPEED = 5.431_rad_per_s;
        constexpr radians_per_second_squared_t MAX_ROTATION_ACCELARATION = 2_rad_per_s_sq;

        constexpr inch_t POSITION_TOLERANCE = 2_in; // Drive to a position, when safe to quit
        constexpr degree_t ANGLE_TOLERANCE = 2_deg;

        namespace PathDrivePIDConstants {
            constexpr double P = 5.0;
            constexpr double I = 0.0;
            constexpr double D = 0.0;
        }

        namespace PathRotationPIDConstants {
            constexpr double P = 2.0;
            constexpr double I = 0.0;
            constexpr double D = 0.0;
        }
    }



}

#endif