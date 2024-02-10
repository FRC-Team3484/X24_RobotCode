#ifndef CONSTANTS_H
#define CONSTANTS_H

#define EN_DIAGNOSTICS
// #define EN_TESTING

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
// #include <ctre/Phoenix.h>

#include <FRC3484_Lib/utils/SC_ControllerMaps.h>
#include <FRC3484_Lib/utils/SC_Datatypes.h>

namespace LauncherConstants {
    constexpr int LEFT_MOTOR_CAN_ID = 40;
    constexpr int RIGHT_MOTOR_CAN_ID = 41;

    constexpr SC::SC_PIDConstants PID_CONSTANTS(0.1, 1e-4, 1, 0);
    constexpr double GEAR_RATIO = 3;
    constexpr double RPM_WINDOW_RANGE = 50;

    //constexpr bool IsLoaded = true;
    constexpr bool MOTOR_INVERTED = true;

    constexpr units::revolutions_per_minute_t TARGET_RPM/*place holder*/ = 1503_rpm;
    constexpr units::revolutions_per_minute_t REVERSE_RPM = -(TARGET_RPM/2); // make a command that tuns this value to rue an drunss the command 
}
namespace IntakeConstants {
    constexpr int PIVOT_MOTOR_CAN_ID = 30;
    constexpr int DRIVE_MOTOR_CAN_ID = 32;
    constexpr int PIECE_SENSOR_DI_CH = 0;
    constexpr int ARM_SENSOR_DI_CH = 1;
    constexpr double GEAR_RATIO = 62.5;

    constexpr units::degrees_per_second_t MAX_VELOCITY = 10_deg_per_s;
    constexpr units::degrees_per_second_squared_t MAX_ACCELERATION = 10_deg_per_s_sq;

    constexpr units::degree_t STOW_POSITION = 0_deg;
    constexpr units::degree_t INTAKE_POSITION = 180_deg;
    constexpr units::degree_t EJECT_POSITION = 180_deg;

    constexpr double HOME_POWER = -0.2;

    constexpr SC::SC_PIDConstants PID_CONSTANTS(0.1, 1e-4, 1, 0);
    constexpr double PID_IZ_ZONE = 0;
    constexpr double PID_OUTPUTRANGE_MIN = -1;
    constexpr double PID_OUTPUTRANGE_MAX = 1;

    constexpr int ROLLER_STOP = 0;
    constexpr double ROLLER_POWER = 0.8;

    constexpr units::degree_t POSITION_TOLERANCE = 2_deg;
}

namespace ClawConstants {
    constexpr int LEFT_MOTOR_CAN_ID = 50;
    constexpr int RIGHT_MOTOR_CAN_ID = 51;
    constexpr int LEFT_SENSOR_DI_CH = 3;
    constexpr int RIGHT_SENSOR_DI_CH = 4;

    constexpr bool MOTOR_INVERTED = false;
    constexpr int MOTOR_STOP = 0;

    constexpr double MOTOR_UP_SPEED = 0.5;
    constexpr double MOTOR_DOWN_SPEED = -0.5;
}

namespace SwerveConstants {
    namespace AutonNames {
    const std::string AUTON_NONE = "Nothing";
    const std::string AUTON_DISTANCE = "Drive 5 feet";
    const std::string AUTON_ANGLE = "Turn 90 degrees";
    const std::string AUTON_SEQUENCE = "Drive Sequence";
    }

    namespace ControllerConstants {
        constexpr double DRIVER_RUMBLE_HIGH = 0.5;
        constexpr double DRIVER_RUMBLE_LOW = 0.2;

        constexpr double OPERATOR_RUMBLE_HIGH = 0.5;
        constexpr double OPERATOR_RUMBLE_LOW = 0.2;
        
        constexpr double RUMBLE_STOP = 0;

    }

    namespace DrivetrainConstants {
        // Swerve Module Configurations

        // For those with static, do not change into constants; it will break the linking
        // DO NOT REMOVE STATIC CALLS

        // Drive, steer, encoder, magnet offset
        static SC::SC_SwerveConfigs SWERVE_FRONT_LEFT{10,11,20, 4.394};
        static SC::SC_SwerveConfigs SWERVE_FRONT_RIGHT{12,13,21,71.630};
        static SC::SC_SwerveConfigs SWERVE_BACK_LEFT{14,15,22,-26.103};
        static SC::SC_SwerveConfigs SWERVE_BACK_RIGHT{16,17,23,-71.455};

        static SC::SC_SwerveConfigs SWERVE_CONFIGS_ARRAY[4] = {
            SWERVE_FRONT_LEFT,
            SWERVE_FRONT_RIGHT,
            SWERVE_BACK_LEFT,
            SWERVE_BACK_RIGHT
        };

        #define FL 0
        #define FR 1
        #define BL 2
        #define BR 3

        constexpr bool STEER_MOTOR_REVERSED = false;
        constexpr bool ENCODER_REVERSED = false;

        constexpr double ENCODER_OFFSET[] = {4.394, 71.630, -26.103, -71.455};

        constexpr double DRIVE_CURRENT_THRESHOLD = 60;
        constexpr double DRIVE_CURRENT_TIME = 0.1;
        constexpr double STEER_CURRENT_THRESHOLD = 40;
        constexpr double STEER_CURRENT_TIME = 0.1;

        constexpr units::inch_t DRIVETRAIN_WIDTH = 24_in;
        constexpr units::inch_t DRIVETRAIN_LENGTH = 24_in;
        constexpr double DRIVE_GEAR_RATIO = 36000.0/5880.0;
        constexpr double STEER_GEAR_RATIO = 12.8;
        constexpr int DRIVE_TICKS_PER_REVOLUTION = 2048;
        constexpr int STEER_TICKS_PER_REVOLUTION = 2048;
        constexpr units::inch_t WHEEL_RADIUS = 2_in;

        constexpr units::feet_per_second_t MAX_WHEEL_SPEED = 8_fps;
        constexpr units::feet_per_second_squared_t MAX_WHEEL_ACCELERATION = 4_fps_sq;

        namespace DrivePIDConstants {
            constexpr double Kp_Drive = 1.0;
            constexpr double Ki_Drive = 0.0;
            constexpr double Kd_Drive = 0.0;
        }
        namespace DriveFeedForwardConstants {
            constexpr units::volt_t S = 1.0_V;
            constexpr auto V = 0.8_V / 1.0_mps;
            constexpr auto A = 0.15_V / 1.0_mps_sq;
        }
        namespace SteerPIDConstants {
            constexpr double Kp_Drive = 0.5;
            constexpr double Ki_Drive = 0.0;
            constexpr double Kd_Drive = 0.0;
            constexpr units::radians_per_second_t MAX_SPEED = 12_rad_per_s;
            constexpr units::radians_per_second_squared_t MAX_ACCELERATION = 100_rad_per_s_sq;
        }

        namespace JoystickScaling {
            constexpr double LOW_SCALE = 0.2;
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
namespace VisionConstants {
    constexpr units::inch_t MAX_LAUNCH_RANGE = 1000_in;
    constexpr units::inch_t AIM_TOLERANCE_LARGE = 12_in;
    constexpr units::inch_t AIM_TOLERANCE_SMALL = 6_in;
    constexpr double CAMERA_ANGLE = 30.0;
    constexpr double CAMERA_HEIGHT = 9.0;
    constexpr double STEER_GAIN = -.01;
    constexpr double TARGET_HEIGHT = 36; // inches
    // multiplier to give how far off and results to a steer power
}
namespace UserInterface {
    namespace Driver {
        constexpr int DRIVER_CONTROLLER_PORT = 0;
        constexpr double JOYSTICK_DEADBAND = 0.02;
        constexpr int THROTTLE = XBOX_LS_Y;
        constexpr int STRAFE = XBOX_LS_X;
        constexpr int ROTATION = XBOX_RS_X;
        constexpr int RESET_HEADING = XBOX_START;
        constexpr int BRAKE = XBOX_X;
        constexpr int BRAKE_MODE = XBOX_RB;
        constexpr int DISABLE_BRAKE_MODE = XBOX_LB;
        constexpr int LOW_SPEED = XBOX_LB;
    }
    namespace Operator {
        constexpr int OPERATOR_CONTROLLER_PORT = 1;
        constexpr int INTAKE_SHOOTER = XBOX_Y;
        constexpr int IGNORE = XBOX_RB;
        constexpr int AIM_START = XBOX_X;
        constexpr int EXTEND = XBOX_A;
        constexpr int EJECT = XBOX_B;
    }
    namespace Testing {
        constexpr int TESTING_CONTROLLER_PORT = 3;
        namespace Hotkey {
            constexpr int LAUNCHER_HK = XBOX_LB;
            constexpr int INTAKE_HK = XBOX_RB;
            constexpr int CLIMBER_HK = XBOX_LT;
        }
        namespace Launcher {
        constexpr int LEFT_MOTOR = XBOX_X;
        constexpr int RIGHT_MOTOR = XBOX_B;
        constexpr int SHOT_SENSOR = XBOX_A;
        }
        namespace Intake {
        constexpr int STOW_ANGLE = XBOX_X;
        constexpr int READY_ANGLE = XBOX_B;
        constexpr int ROLL_FORWARD = XBOX_Y;
        constexpr int ROLL_BACKWARD = XBOX_A;
        constexpr int HAS_PIECE_SENSOR = XBOX_RT;
        }
    }
}

#endif
