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
// frontLeft, frontRight, backLeft, backRight
frc::SwerveDriveKinematics<4> kDriveKinematics{
  frc::Translation2d{+0.2906_m, +0.2876_m},
  frc::Translation2d{+0.2896_m, -0.2896_m},
  frc::Translation2d{-0.2894_m, +0.2884_m},
  frc::Translation2d{-0.2884_m, -0.2926_m}
};
frc::Rotation2d zero{0_rad};



frc::Translation2d kStartTranslation{2.0_m, 2.0_m};
frc::Rotation2d kStartRotation{0.0_rad};
frc::Pose2d kStartPose {kStartTranslation, kStartRotation};

frc::SwerveDriveOdometry<4> m_odo {kDriveKinematics, zero, positions, kStartPose};


SwerveModule* m_frontLeft;
SwerveModule* m_frontRight;
SwerveModule* m_backLeft;
SwerveModule* m_backRight; 

SwerveModule** m_modules;

inline static AHRS* m_imu = new AHRS(frc::I2C::Port::kMXP);

pathplanner::PIDConstants kPPTranslationPID{0.75};
pathplanner::PIDConstants kPPRotationPID{0};
inline static constexpr units::length::meter_t kDriveRadius{0.408};

frc::Rotation2d angle;
frc::SwerveModulePosition position;
frc::Pose2d pose;
wpi::array<frc::SwerveModulePosition, 4> positions {position, position, position, position};
frc::ChassisSpeeds speeds;
frc::SwerveModuleState state;
wpi::array<frc::SwerveModuleState, 4> states{state, state, state, state};


frc::Rotation2d GetAngle();

void ResetHeading();

void ImuBoot();

wpi::array<frc::SwerveModulePosition, kNumModules> GetPositions();

frc::Pose2d GetPose();

void ResetPose();

void ResetPose(frc::Pose2d pose);

frc::ChassisSpeeds GetChassisSpeeds();

void DriveRobotRelative(frc::ChassisSpeeds speeds);

frc2::InstantCommand* resetHeadingCommand = new frc2::InstantCommand{[this]() {return this->ResetHeading();}}; 
frc2::InstantCommand* resetPoseCommand = new frc2::InstantCommand{[this]() {return this->ResetPose();}}; 

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
