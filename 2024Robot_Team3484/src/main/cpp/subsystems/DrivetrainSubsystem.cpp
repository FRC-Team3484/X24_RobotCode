#include "subsystems/DrivetrainSubsystem.h"

#include <frc/geometry/Translation2d.h>

#include <frc/smartdashboard/SmartDashboard.h>

// #include <pathplanner/lib/path/PathPlannerPath.h>

using namespace frc;
using namespace units;
using namespace SwerveConstants::DrivetrainConstants;

DrivetrainSubsystem::DrivetrainSubsystem() {
    _gyro = new AHRS{SPI::Port::kMXP};
    _odometry = new SwerveDriveOdometry<4>{kinematics, GetHeading(), GetModulePositions()};
}

void DrivetrainSubsystem::Periodic() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accessed in Periodic before initialization");
    } else {
        _odometry->Update(GetHeading(), GetModulePositions());
    }
}

void DrivetrainSubsystem::Drive(meters_per_second_t x_speed, meters_per_second_t y_speed, radians_per_second_t rotation, bool open_loop) {
    auto states = kinematics.ToSwerveModuleStates(ChassisSpeeds::FromFieldRelativeSpeeds(x_speed, y_speed, rotation, GetHeading()));

    SetModuleStates(states, open_loop, true);
}

void DrivetrainSubsystem::SetModuleStates(wpi::array<SwerveModuleState, 4> desired_states, bool open_loop, bool optimize) {
    kinematics.DesaturateWheelSpeeds(&desired_states, MAX_WHEEL_SPEED);
    for (int i = 0; i < 4; i++) {
        _modules[i].SetDesiredState(desired_states[i], open_loop, optimize);
    }
}

Rotation2d DrivetrainSubsystem::GetHeading() {
    if (_gyro == NULL) {
        fmt::print("Error: gyro accessed in GetHeading before initialization");
        return Rotation2d{0_deg};
    } else {
        return degree_t{-_gyro->GetAngle()} + _gyro_offset;
    }
}

void DrivetrainSubsystem::SetHeading(degree_t heading) {
    if (_gyro == NULL) {
        fmt::print("Error: gyro accessed in ZeroHeading before initialization");
    } else {
        _gyro_offset = heading;
        _gyro->ZeroYaw();
        ResetOdometry(GetPose());
    }
    
}

degrees_per_second_t DrivetrainSubsystem::GetTurnRate() {
    if (_gyro == NULL) {
        fmt::print("Error: gyro accessed in GetTurnRate before initialization");
        return 0_deg_per_s;
    } else {
        return degrees_per_second_t{_gyro->GetRate()};
    }
}

Pose2d DrivetrainSubsystem::GetPose() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accesed in GetPose before initialization");
        return Pose2d{0_m, 0_m, 0_deg};
    } else {
        return _odometry->GetPose();
    }
}

void DrivetrainSubsystem::ResetOdometry(Pose2d pose) {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accesed in ResetOdometry before initialization");
    } else {
        _odometry->ResetPosition(GetHeading(), GetModulePositions(), pose);
    }
    
}

wpi::array<SwerveModulePosition, 4> DrivetrainSubsystem::GetModulePositions() {
    return {_modules[FL].GetPosition(), _modules[FR].GetPosition(), _modules[BL].GetPosition(), _modules[BR].GetPosition()};
}

ChassisSpeeds DrivetrainSubsystem::GetChassisSpeeds() {
    return kinematics.ToChassisSpeeds({_modules[FL].GetState(), _modules[FR].GetState(), _modules[BL].GetState(), _modules[BR].GetState()});
}

void DrivetrainSubsystem::StopMotors() {
    _modules[FL].StopMotors();
    _modules[FR].StopMotors();
    _modules[BL].StopMotors();
    _modules[BR].StopMotors();
}

void DrivetrainSubsystem::ResetEncoders() {
    _modules[FL].ResetEncoder();
    _modules[FR].ResetEncoder();
    _modules[BL].ResetEncoder();
    _modules[BR].ResetEncoder();
    ResetOdometry(GetPose());
}

void DrivetrainSubsystem::SetCoastMode() {
    _modules[FL].SetCoastMode();
    _modules[FR].SetCoastMode();
    _modules[BL].SetCoastMode();
    _modules[BR].SetCoastMode();
}
void DrivetrainSubsystem::SetBrakeMode() {
    _modules[FL].SetBrakeMode();
    _modules[FR].SetBrakeMode();
    _modules[BL].SetBrakeMode();
    _modules[BR].SetBrakeMode();
}