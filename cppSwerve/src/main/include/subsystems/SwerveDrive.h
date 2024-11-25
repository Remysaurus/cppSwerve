// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include "Constants.h"
#include "SwerveModule.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <AHRS.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>


class SwerveDrive : public frc2::SubsystemBase {




public:
  SwerveDrive();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
constexpr inline static int kNumModules = 4;
frc::SwerveDriveKinematics<kNumModules>* kDriveKinematics;
frc::SwerveDriveOdometry<kNumModules>* m_odo;
frc::Translation2d* kStartTranslation = new frc::Translation2d{2.0_m, 2.0_m};


SwerveModule* m_frontLeft;
SwerveModule* m_frontRight;
SwerveModule* m_backLeft;
SwerveModule* m_backRight; 

SwerveModule** m_modules;

inline static AHRS* m_imu = new AHRS(frc::I2C::Port::kMXP); //check if this is right

pathplanner::PIDConstants kPPTranslationPID{5.0};
pathplanner::PIDConstants kPPRotationPID{5.0};
inline static constexpr units::length::meter_t kDriveRadius{0.408};

frc::Rotation2d GetAngle();

void ResetHeading();

void ImuBoot();

wpi::array<frc::SwerveModulePosition, kNumModules> GetPositions();

frc::Pose2d GetPose();

void ResetPose(frc::Pose2d pose);

frc::ChassisSpeeds GetChassisSpeeds();

void DriveRobotRelative(frc::ChassisSpeeds speeds);

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
