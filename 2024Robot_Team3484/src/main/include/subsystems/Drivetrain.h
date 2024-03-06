// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#ifndef DRIVE
#define DRIVE

#include <functional>

#include <frc/Encoder.h>
#include <frc/RobotController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/length.h>

#include <frc/controller/PIDController.h>

// #include "ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/WPI_CANCoder.h"



class Drive : public frc2::SubsystemBase {
 public:
  Drive();

  frc2::CommandPtr PseudoDriveCommand(std::function<double()> fwd,
                                      std::function<double()> rot);
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);
  
  void Periodic() override;
  // Steer Angles
  units::degree_t GetSteerAngleFL();
  units::degree_t GetSteerAngleFR();
  units::degree_t GetSteerAngleBL();
  units::degree_t GetSteerAngleBR();
  

 private:
//   frc::PWMSparkMax m_leftMotor{constants::drive::kLeftMotor1Port};
//   frc::PWMSparkMax m_rightMotor{constants::drive::kRightMotor1Port};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX _drive_motor_FL{10};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX _drive_motor_FR{12};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX _drive_motor_BL{14};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX _drive_motor_BR{16};

    ctre::phoenix::motorcontrol::can::WPI_TalonFX _steer_motor_FL{11};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX _steer_motor_FR{13};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX _steer_motor_BL{15};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX _steer_motor_BR{17};

    ctre::phoenix::sensors::WPI_CANCoder _encoder_FL{20};
    ctre::phoenix::sensors::WPI_CANCoder _encoder_FR{21};
    ctre::phoenix::sensors::WPI_CANCoder _encoder_BL{22};
    ctre::phoenix::sensors::WPI_CANCoder _encoder_BR{23};

    // PID Loops

    // frc::PIDController _drive_pid_controller{1.0,0.0,0.0};

    frc::ProfiledPIDController<units::radians> _steer_pid_controller_FL{.5, 0.0, 0.0, 
        {12_rad_per_s, 100_rad_per_s_sq}};
    frc::ProfiledPIDController<units::radians> _steer_pid_controller_FR{.5, 0.0, 0.0, 
        {12_rad_per_s, 100_rad_per_s_sq}};
    frc::ProfiledPIDController<units::radians> _steer_pid_controller_BL{.5, 0.0, 0.0, 
        {12_rad_per_s, 100_rad_per_s_sq}};
    frc::ProfiledPIDController<units::radians> _steer_pid_controller_BR{.5, 0.0, 0.0, 
        {12_rad_per_s, 100_rad_per_s_sq}};
    ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration _steer_current_limit{
        true, 25, 40, 0.1
    };

frc::DifferentialDrive m_drive{[this](auto val) { _drive_motor_FL.Set(val); },
                                [this](auto val) { _drive_motor_FR.Set(val); }
};


  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
            _drive_motor_FL.SetVoltage(driveVoltage);
            _drive_motor_FR.SetVoltage(driveVoltage);
            // _drive_motor_BL.SetVoltage(driveVoltage);
            // _drive_motor_BR.SetVoltage(driveVoltage);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("drive-fl")
                .voltage(_drive_motor_FL.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{_drive_motor_FL.GetSelectedSensorPosition()*600/(36000.0/5880.0)})
                .velocity(units::meters_per_second_t{_drive_motor_FL.GetSelectedSensorVelocity()*600/(36000.0/5880.0)});
            log->Motor("drive-fr")
                .voltage(_drive_motor_FR.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{_drive_motor_FR.GetSelectedSensorPosition()*600/(36000.0/5880.0)})
                .velocity(units::meters_per_second_t{_drive_motor_FR.GetSelectedSensorVelocity()*600/(36000.0/5880.0)});
            log->Motor("drive-bl")
                .voltage(_drive_motor_BL.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{_drive_motor_BL.GetSelectedSensorPosition()*600/(36000.0/5880.0)})
                .velocity(units::meters_per_second_t{_drive_motor_BL.GetSelectedSensorVelocity()*600/(36000.0/5880.0)});
            log->Motor("drive-br")
                .voltage(_drive_motor_BR.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{_drive_motor_BR.GetSelectedSensorPosition()*600/(36000.0/5880.0)})
                .velocity(units::meters_per_second_t{_drive_motor_BR.GetSelectedSensorVelocity()*600/(36000.0/5880.0)});
          },
          this}};
};





#endif

