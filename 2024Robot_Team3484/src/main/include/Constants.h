#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <ctre/Phoenix.h>



#include <FRC3484_Lib/utils/SC_ControllerMaps.h>



namespace ShooterConstants {}
namespace IntakeConstants {}
namespace HookConstants {}








namespace SwerveConstants {
    namespace AutonNames {
    const std::string AUTON_NONE = "Nothing";
    const std::string AUTON_DISTANCE = "Drive 5 feet";
    const std::string AUTON_ANGLE = "Turn 90 degrees";
    const std::string AUTON_SEQUENCE = "Drive Sequence";
    }

    namespace ControllerConstants {
        namespace Driver {
            constexpr int DRIVER_CONTROLLER_PORT = 0;
            constexpr double JOYSTICK_DEADBAND = 0.02;
            #define THROTTLE XBOX_LS_Y 
            #define STRAFE XBOX_LS_X
            #define ROTATION XBOX_RS_X
            #define RESET_HEADING XBOX_BACK
            #define BRAKE XBOX_X
            #define STRAIGHTEN_WHEELS XBOX_START
            #define BRAKE_MODE XBOX_RB
            #define DISABLE_BRAKE_MODE XBOX_LB
        }
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

        const motorcontrol::SupplyCurrentLimitConfiguration DRIVE_CURRENT_LIMIT{true, 35, 60, 0.1};
        const motorcontrol::SupplyCurrentLimitConfiguration STEER_CURRENT_LIMIT{true, 25, 40, 0.1};

        constexpr double ENCODER_OFFSET[] = {4.394, 71.630, -26.103, -71.455};

        constexpr double DRIVE_CURRENT_THRESHOLD = 60;
        // constexpr double DRIVE_CURRENT_LIMIT = 35;
        constexpr double DRIVE_CURRENT_TIME = 0.1;
        constexpr double STEER_CURRENT_THRESHOLD = 40;
        // constexpr double STEER_CURRENT_LIMIT = 25;
        constexpr double STEER_CURRENT_TIME = 0.1;

        constexpr units::inch_t DRIVETRAIN_WIDTH = 24_in;
        constexpr units::inch_t DRIVETRAIN_LENGTH = 24_in;
        constexpr double DRIVE_GEAR_RATIO = 36000.0/5880.0;
        constexpr double STEER_GEAR_RATIO = 12.8;
        constexpr int DRIVE_TICKS_PER_REVOLUTION =2048;
        constexpr int STEER_TICKS_PER_REVOLUTION = 2048;
        constexpr units::inch_t WHEEL_RADIUS = 2_in;

        constexpr units::feet_per_second_t MAX_WHEEL_SPEED = 8_fps;
        constexpr units::feet_per_second_squared_t MAX_WHEEL_ACCELERATION = 4_fps_sq;

        namespace DrivePIDConstants {
            constexpr double Kp_Drive = 1.0;
            constexpr double Ki_Drive = 0.0;
            constexpr double Kd_Drive = 0.0;
        }
        namespace DriveFeedForwardConstants { //Future Check
            constexpr units::volt_t S = 1.0_V;
            constexpr auto V = 0.8_V / 1.0_mps;
            constexpr auto A = 0.15_V / 1.0_mps_sq;
        }
        namespace SteerPIDConstants {
            constexpr double Kp_Drive = 0.5;
            constexpr double Ki_Drive = 0.0;
            constexpr double Kd_Drive = 0.0;
            constexpr units::radians_per_second_t MAX_SPEED = 12_rad_per_s;
            constexpr units::radians_per_second_squared_t MAX_ACCELERATION  = 100_rad_per_s_sq;
        }
    }

    namespace BrakeConstants {
        constexpr auto DYNAMIC_BRAKE_SCALING = -00.2/1_in;
    }

    namespace AutonDriveConstants {
        // How fast the robot can move in autons
        constexpr units::feet_per_second_t MAX_LINEAR_SPEED = 8_fps;
        constexpr units::feet_per_second_squared_t MAX_LINEAR_ACCELERATION = 4_fps_sq;
        constexpr units::radians_per_second_t MAX_ROTATION_SPEED = 5.431_rad_per_s;
        constexpr units::radians_per_second_squared_t MAX_ROTATION_ACCELERATION = 2_rad_per_s_sq;

        constexpr units::inch_t POSITION_TOLERANCE = 2_in; // Drive to a position, when safe to quit
        constexpr units::degree_t ANGLE_TOLERANCE = 2_deg;

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