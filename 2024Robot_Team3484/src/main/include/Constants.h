#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "units/time.h"
#define EN_TESTING

#define CLIMBER_ENABLED
// #define TRAP_ENABLED
#define INTAKE_ENABLED
#define DRIVE_ENABLED
#define AIM_ENABLED
#define LAUNCHER_ENABLED

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

// #include <ctre/Phoenix.h>

#include <FRC3484_Lib/utils/SC_ControllerMaps.h>
#include <FRC3484_Lib/utils/SC_Datatypes.h>

namespace LauncherConstants {
    constexpr int LEFT_MOTOR_CAN_ID = 40;
    constexpr int RIGHT_MOTOR_CAN_ID = 41;
    constexpr int LAUNCH_SENSOR_DI_CH = 2; // Change to 2


    constexpr SC::SC_PIDConstants LEFT_PID_CONSTANTS(5e-5, 5e-7, 0, 0);
    constexpr SC::SC_PIDConstants RIGHT_PID_CONSTANTS(5e-5, 5e-7, 0, 1.6e-4);
    //7e-8
    constexpr double GEAR_RATIO = 3;
    constexpr double RPM_WINDOW_RANGE = 50;
    constexpr units::second_t WINDOW_TIME = .25_s;
    // Set logic as if hit -50 window, may run too early

    //constexpr bool IsLoaded = true;
    constexpr bool LEFT_MOTOR_INVERTED = false;
    
    // Target RPM
    constexpr units::revolutions_per_minute_t TARGET_RPM/*place holder*/ = 1600_rpm;
    constexpr units::revolutions_per_minute_t REVERSE_RPM = -300_rpm; // make a command that tuns this value to rue an drunss the command 
    constexpr units::revolutions_per_minute_t AMP_RPM = 850_rpm;
    constexpr units::revolutions_per_minute_t TRAP_RPM = 750_rpm;
}
namespace IntakeConstants {
    constexpr int PIVOT_MOTOR_CAN_ID = 30;
    constexpr int DRIVE_MOTOR_CAN_ID = 31;
    constexpr int PIECE_SENSOR_DI_CH = 0; // Change to 0
    constexpr int ARM_SENSOR_DI_CH = 1;
    constexpr double GEAR_RATIO = 100.0/9.0;

    constexpr units::degrees_per_second_t MAX_VELOCITY = 250_deg_per_s;
    constexpr units::degrees_per_second_squared_t MAX_ACCELERATION = 1000_deg_per_s_sq;

    constexpr units::degree_t STOW_POSITION = 0_deg;
    constexpr units::degree_t INTAKE_POSITION = 174_deg;
    constexpr units::degree_t EJECT_POSITION = 90_deg;

    constexpr double HOME_POWER = -0.25; 

    constexpr SC::SC_PIDConstants PIVOT_PID_CONSTANTS(0.4, 0.0005, 0, 0);
    constexpr double PID_IZ_ZONE = 0;
    constexpr double PID_OUTPUTRANGE_MIN = -1;
    constexpr double PID_OUTPUTRANGE_MAX = 1;

    constexpr int ROLLER_STOP = 0;
    constexpr double ROLLER_POWER = 0.8;
    constexpr double INTAKE_SHOOTER_POWER = 0.4;

    constexpr units::degree_t POSITION_TOLERANCE = 5_deg;

    constexpr uint PIVOT_STALL_LIMIT = 60;
    constexpr uint PIVOT_FREE_LIMIT = 40;
    constexpr uint DRIVE_STALL_LIMIT = 60;
    constexpr uint DRIVE_FREE_LIMIT = 40;
}

namespace ClimberConstants {
    constexpr int LEFT_MOTOR_CAN_ID = 50;
    constexpr int RIGHT_MOTOR_CAN_ID = 51;
    constexpr int LEFT_SENSOR_DI_CH = 3;
    constexpr int RIGHT_SENSOR_DI_CH = 4;

    constexpr bool MOTOR_INVERTED = false;
    constexpr int MOTOR_STOP = 0;

    constexpr double MOTOR_UP_SPEED = -0.5;
    constexpr double MOTOR_DOWN_SPEED = 0.5;
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
        static SC::SC_SwerveConfigs SWERVE_FRONT_LEFT{10,11,20, -92.505};
        static SC::SC_SwerveConfigs SWERVE_FRONT_RIGHT{12,13,21,-60.205};
        static SC::SC_SwerveConfigs SWERVE_BACK_LEFT{14,15,22,160.654};
        static SC::SC_SwerveConfigs SWERVE_BACK_RIGHT{16,17,23,-55.283};

        constexpr units::second_t X_BRAKE_TIMER = .5_s;

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
            constexpr double Kp_Steer = 0.5;
            constexpr double Ki_Steer = 0.0;
            constexpr double Kd_Steer = 0.0;
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
    constexpr double CAMERA_ANGLE = 38.0;
    constexpr double CAMERA_HEIGHT = 22.5;
    constexpr double STEER_GAIN = -.01;
    constexpr double TARGET_HEIGHT = 57; // inches
    // multiplier to give how far off and results to a steer power
}

namespace TrapConstants {
    constexpr int EXTENSION_MOTOR_CAN_ID = 60;
    constexpr int GP_CONTROL_CAN_ID = 61;
    constexpr bool MOTOR_INVERTED = false;
    constexpr units::feet_per_second_t MAX_VELOCITY = 0.5_fps;
    constexpr units::feet_per_second_squared_t MAX_ACCELERATION = 0.1_fps_sq;
    constexpr units::inch_t HOME_POSITION = 0_in;
    constexpr units::inch_t INTAKE_POSITION = 6_in;
    constexpr units::inch_t TRAP_POSITION = 12_in;
    constexpr units::inch_t AMP_POSITION = 8_in;
    constexpr double EJECT_POWER = -0.5;
    constexpr double INTAKE_POWER = 0.5;
    constexpr SC::SC_PIDConstants PID_CONSTANTS(0.1, 1e-4, 1, 0);
    constexpr units::inch_t POSITION_TOLORANCE = 1_in;
    constexpr units::inch_t GEAR_RATIO = 1.0_in;
    constexpr double PID_MAX = 1;
    constexpr double PID_MIN = -1;

}
namespace UserInterface {
    namespace Driver {
        constexpr int DRIVER_CONTROLLER_PORT = 0;
        constexpr double DRIVER_JOYSTICK_DEADBAND = 0.02;
        // Motion
        constexpr int THROTTLE = XBOX_LS_Y;
        constexpr int STRAFE = XBOX_LS_X;
        constexpr int ROTATION = XBOX_RS_X;
        // Settings
        constexpr int RESET_HEADING = XBOX_BACK;
        constexpr int BRAKE = XBOX_X;
        constexpr int BRAKE_MODE = XBOX_RB;
        constexpr int DISABLE_BRAKE_MODE = XBOX_LB;
        constexpr int LOW_SPEED = XBOX_LT;

        // Ignore
        constexpr int DRIVER_IGNORE = XBOX_Y;

    }
    namespace Operator {
        constexpr int OPERATOR_CONTROLLER_PORT = 1;
        // Intake
        constexpr int OPERATOR_IGNORE = XBOX_RB;
        constexpr int EXTEND = XBOX_A;
        constexpr int EJECT = XBOX_B;
        constexpr int LAUNCHER_TRAP = XBOX_X;
        constexpr int LAUNCHER_AMP = XBOX_Y;
        constexpr int LAUNCHER_INTAKE = XBOX_A; // In accordance with INTAKE_LAUNCHER

        //Launcher
        constexpr int LAUNCHER_TOGGLE_HK = XBOX_LT;
        constexpr int INTAKE_LAUNCHER = XBOX_Y;
        constexpr int AIM_START = XBOX_X;

        // Climb
        // D-Pad: Hard Coded in OI

        // Trap
        constexpr int ENDGAME_TOGGLE_HK = XBOX_LB;
        constexpr int INTAKE_TRAP = XBOX_A;
        constexpr int SCORE_TRAP = XBOX_X;
        constexpr int AMP_TRAP = XBOX_Y;
    }
    namespace Testing {
        constexpr int TESTING_OPEN_LOOP_LEFT = XBOX_LS_Y;
        constexpr int TESTING_OPEN_LOOP_RIGHT = XBOX_RS_Y;
        constexpr double TESTING_DEADBAND = 0.1;
        constexpr int TESTING_CONTROLLER_PORT = 3;
        constexpr int TOGGLE_TESTING = XBOX_BACK;
        namespace Hotkey {
            constexpr int LAUNCHER_HK = XBOX_LB;
            constexpr int INTAKE_HK = XBOX_RB;
            constexpr int CLIMBER_HK = XBOX_LT;
            constexpr int TRAP_HK = XBOX_RT;
        }
    }
}
#endif
