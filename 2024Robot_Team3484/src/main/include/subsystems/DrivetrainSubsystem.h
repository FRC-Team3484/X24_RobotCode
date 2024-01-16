#ifndef DRIVETRAINSUBSYSTEM_H
#define DRIVETRAINSUBSYSTEM_H

#include "Constants.h"
#include "SwerveModule.h"

#include <units/angle.h>
#include <units/angular_velocity.h>

#include <AHRS.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

class DrivetrainSubsystem : public frc2::SubsystemBase {
    public:
        DrivetrainSubsystem();
        void Periodic() override;

        void Drive(units::meters_per_second_t x_speed, units::meters_per_second_t y_speed, units::radians_per_second_t rotation, bool open_loop=false);
        void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desired_states, bool open_loop=false, bool optimize=true);
        frc::Rotation2d GetHeading();
        void SetHeading(units::degree_t heading=0_deg);
        units::degrees_per_second_t GetTurnRate();
        frc::Pose2d GetPose();
        void ResetOdometry(frc::Pose2d pose);
        wpi::array<frc::SwerveModulePosition, 4> GetModulePositions();
        frc::ChassisSpeeds GetChassisSpeeds();
        void StopMotors();
        void ResetEncoders();
        void SetCoastMode();
        void SetBrakeMode();

        frc::SwerveDriveKinematics<4> kinematics{
            frc::Translation2d{SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, -SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2},
            frc::Translation2d{-SwerveConstants::DrivetrainConstants::DRIVETRAIN_LENGTH/2, -SwerveConstants::DrivetrainConstants::DRIVETRAIN_WIDTH/2}
        };

    private:
        SwerveModule _modules[4] = {
            SwerveModule{FL},
            SwerveModule{FR},
            SwerveModule{BL},
            SwerveModule{BR}
        };

        AHRS* _gyro;
        units::degree_t _gyro_offset = 0_deg;

        frc::SwerveDriveOdometry<4>* _odometry;
};

#endif