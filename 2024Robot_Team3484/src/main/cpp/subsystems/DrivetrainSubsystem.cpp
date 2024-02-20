#include "subsystems/DrivetrainSubsystem.h"

#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace units;
using namespace SwerveConstants::DrivetrainConstants;
using namespace SC;

DrivetrainSubsystem::DrivetrainSubsystem(SC_SwerveConfigs swerve_config_array[4]) {
    if (NULL != swerve_config_array) {
        for (int i = 0; i < 4; i++) {
            _modules[i] = new SwerveModule(swerve_config_array[i]);
        }
    }
    _gyro = new AHRS{SPI::Port::kMXP};
    _odometry = new SwerveDriveOdometry<4>{kinematics, GetHeading(), GetModulePositions()};
}

void DrivetrainSubsystem::Periodic() {
    if (_odometry == NULL) {
        fmt::print("Error: odometry accessed in Periodic before initialization");
    } else {
        _odometry->Update(GetHeading(), GetModulePositions());
    }
    #ifdef EN_DIAGNOSTICS
        SmartDashboard::PutNumber("FL Encoder", _modules[FL]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("FR Encoder", _modules[FR]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("BL Encoder", _modules[BL]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("BR Encoder", _modules[BR]->GetPosition().angle.Degrees().value());
        SmartDashboard::PutNumber("Gyro Heading", GetHeading().Degrees().value());

    #endif
}

void DrivetrainSubsystem::Drive(meters_per_second_t x_speed, meters_per_second_t y_speed, radians_per_second_t rotation, bool open_loop) {
    auto states = kinematics.ToSwerveModuleStates(ChassisSpeeds::FromFieldRelativeSpeeds(x_speed, y_speed, rotation, GetHeading()));

    SetModuleStates(states, open_loop, true);
}

void DrivetrainSubsystem::SetModuleStates(wpi::array<SwerveModuleState, 4> desired_states, bool open_loop, bool optimize) {
    kinematics.DesaturateWheelSpeeds(&desired_states, MAX_WHEEL_SPEED);
    for (int i = 0; i < 4; i++) {
        if (_modules[i] == NULL) {
            fmt::print("Error: Swerve Module accessed in Periodic before initialization");
        } else {
            _modules[i]->SetDesiredState(desired_states[i], open_loop, optimize);
        }
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
    ResetOdometry(Pose2d(_odometry->GetPose().Translation(), Rotation2d(heading)));
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
    } else if (_gyro == NULL) {
        fmt::print("Error: gyro accessed in ZeroHeading before initialization");
    } else {
        _gyro_offset = pose.Rotation().Degrees();
        _gyro->ZeroYaw();
        _odometry->ResetPosition(GetHeading(), GetModulePositions(), pose);
    }
    
}

wpi::array<SwerveModulePosition, 4> DrivetrainSubsystem::GetModulePositions() {
    int checkNull = CheckNotNullModule();
    if (checkNull == 4)
        return {_modules[FL]->GetPosition(), _modules[FR]->GetPosition(), _modules[BL]->GetPosition(), _modules[BR]->GetPosition()};
    else
        return {SwerveModulePosition(0_m, 0_deg),SwerveModulePosition(0_m, 0_deg),SwerveModulePosition(0_m, 0_deg),SwerveModulePosition(0_m, 0_deg)};
}

ChassisSpeeds DrivetrainSubsystem::GetChassisSpeeds() {
    int checkNull = CheckNotNullModule();
    if (checkNull == 4)
        return kinematics.ToChassisSpeeds({_modules[FL]->GetState(), _modules[FR]->GetState(), _modules[BL]->GetState(), _modules[BR]->GetState()});
    else
        return kinematics.ToChassisSpeeds({SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg), SwerveModuleState(0_mps, 0_deg)});
}

void DrivetrainSubsystem::StopMotors() {
    if (_modules[FL] != NULL) {
        _modules[FL]->StopMotors();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->StopMotors();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->StopMotors();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->StopMotors();
    }
}

void DrivetrainSubsystem::ResetEncoders() {
    if (_modules[FL] != NULL) {
        _modules[FL]->ResetEncoder();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->ResetEncoder();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->ResetEncoder();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->ResetEncoder();
    }
    ResetOdometry(GetPose());
}

void DrivetrainSubsystem::SetCoastMode() {
    if (_modules[FL] != NULL) {
        _modules[FL]->SetCoastMode();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->SetCoastMode();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->SetCoastMode();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->SetCoastMode();
    }
}

void DrivetrainSubsystem::SetBrakeMode() {
    if (_modules[FL] != NULL) {
        _modules[FL]->SetBrakeMode();
    }
    if (_modules[FR] != NULL) {
        _modules[FR]->SetBrakeMode();
    }
    if (_modules[BL] != NULL) {
        _modules[BL]->SetBrakeMode();
    }
    if (_modules[BR] != NULL) {
        _modules[BR]->SetBrakeMode();
    }
}

int DrivetrainSubsystem::CheckNotNullModule() {
    int counter = 0;
    for (int i = 0; i < 4; i++){
        if (_modules[i] != NULL) {
            counter++;
        }
    }
    return counter;
}
