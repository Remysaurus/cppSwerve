// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix/StatusCodes.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>

#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Rotation2d.h>


class SwerveModule : public frc2::SubsystemBase {
  public:
  friend class SwerveDrive;
  

  
  SwerveModule(
    int driveMotorID, 
    int turnMotorID, 
    int turnEncoderPort, 
    double turnEncoderOffset, 
    bool driveInverted, 
    bool turnInverted);

  units::length::meter_t GetDrivePosition();

  units::velocity::meters_per_second_t GetDriveVelocity();

  frc::Rotation2d GetTurnPosition();

  void ResetEncoders();

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void Stop();

  void SetDesiredState(frc::SwerveModuleState state);

  

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  private:
  ctre::phoenix6::hardware::TalonFX* m_driveMotor;
  rev::CANSparkMax* m_turnMotor;

  frc::DutyCycleEncoder* m_turnEncoder;

  frc::PIDController* m_turnPIDController;
  frc::PIDController* m_drivePIDController;

  inline static constexpr double kDriveGearRatio = 5.9; // originally 5.9028 in Java version. 5.9 is the number cited on the SDS website. 
  inline static constexpr double kWheelCircumference = std::numbers::pi * 0.1016 * 0.95; // meters, 0.95 is the wheel correction factor. I am still dubious about it, but that will have to be tested
  inline static constexpr double kDriveDistancePerEncoderRotation = kDriveGearRatio/kWheelCircumference; //how many meters one rotation of the motors moves the robot
  inline static constexpr double kDriveDeadband = 0.01;
  inline static constexpr units::velocity::meters_per_second_t kDriveMaxSpeed{5.761}; // meters per second

  inline static constexpr double kPDrive = 0.17;
  inline static constexpr double kIDrive = 1.7;
  inline static constexpr double kDDrive = 0.0;

  inline static constexpr double kPTurn = 0.5;
  inline static constexpr double kITurn = 0.0;
  inline static constexpr double kDTurn = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
