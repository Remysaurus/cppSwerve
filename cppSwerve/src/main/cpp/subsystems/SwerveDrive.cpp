// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include <thread>
#include "subsystems/SwerveDrive.h"
#include "subsystems/SwerveModule.h"



SwerveDrive::SwerveDrive() {
    std::cout << "Hello World" << std::endl;

    


    

    constexpr std::array<int, kNumModules> kDriveMotorIDs = {51, 52, 53, 54};
    constexpr std::array<int, kNumModules> kTurnMotorIDs = {1, 3, 7, 5};
    constexpr std::array<int, kNumModules> kTurnEncoderPorts = {9, 7, 8, 6};
    constexpr std::array<double, kNumModules> kTurnEncoderOffsets = {
        0,
        0,
        0, 
        0
    };
    constexpr std::array<bool, kNumModules> kDriveReverseds = {true, true, false, true};
    constexpr std::array<bool, kNumModules> kTurnReverseds = {true, true, true, false};

    std::cout << "arrays" << std::endl;

    m_frontLeft = new SwerveModule(
        kDriveMotorIDs[0],
        kTurnMotorIDs[0],
        kTurnEncoderPorts[0],
        kTurnEncoderOffsets[0],
        kDriveReverseds[0],
        kTurnReverseds[0]
    );
    m_frontRight = new SwerveModule(
        kDriveMotorIDs[1],
        kTurnMotorIDs[1],
        kTurnEncoderPorts[1],
        kTurnEncoderOffsets[1],
        kDriveReverseds[1],
        kTurnReverseds[1]
    );
    m_backLeft = new SwerveModule(
        kDriveMotorIDs[2],
        kTurnMotorIDs[2],
        kTurnEncoderPorts[2],
        kTurnEncoderOffsets[2],
        kDriveReverseds[2],
        kTurnReverseds[2]
    );
    m_backRight = new SwerveModule(
        kDriveMotorIDs[3],
        kTurnMotorIDs[3],
        kTurnEncoderPorts[3],
        kTurnEncoderOffsets[3],
        kDriveReverseds[3],
        kTurnReverseds[3]
    );

    std::cout << "modules" << std::endl;

    m_modules = new SwerveModule*[kNumModules] {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

    pathplanner::ReplanningConfig* robotConfig = new pathplanner::ReplanningConfig();
    pathplanner::AutoBuilder::configureHolonomic(
        [this]() {return this->GetPose();},
        [this](const frc::Pose2d& pose) {this->ResetPose(pose);},
        [this]() {return this->GetChassisSpeeds();},
        [this](const frc::ChassisSpeeds& speeds) {this->DriveRobotRelative(speeds);},
        pathplanner::HolonomicPathFollowerConfig(
            kPPTranslationPID,
            kPPRotationPID,
            SwerveModule::kDriveMaxSpeed,
            kDriveRadius,
            robotConfig
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this
    );
    ResetHeading();
    std::cout << "AutoBuilds" << std::endl;
    
}

frc::Rotation2d SwerveDrive::GetAngle() {
    units::angle::radian_t rad{std::remainder(m_imu->GetAngle() * -1, std::numbers::pi)};
    frc::Rotation2d rotation{rad};
    return rotation;
}

void SwerveDrive::ResetHeading() {
    m_imu->Reset();
}

void SwerveDrive::ImuBoot() {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ResetHeading();
}

wpi::array<frc::SwerveModulePosition, 4> SwerveDrive::GetPositions() {
    wpi::array<frc::SwerveModulePosition, 4> positions {
        m_frontLeft->GetPosition(),
        m_frontRight->GetPosition(),
        m_backLeft->GetPosition(),
        m_backRight->GetPosition()
    };
    return positions;
};

frc::Pose2d SwerveDrive::GetPose() {
    return m_odo.GetPose();
}

void SwerveDrive::ResetPose(frc::Pose2d pose) {
    m_odo.ResetPosition(GetAngle(), GetPositions(), pose);
}

frc::ChassisSpeeds SwerveDrive::GetChassisSpeeds() {
    frc::ChassisSpeeds speeds {kDriveKinematics.ToChassisSpeeds(
        m_frontLeft->GetState(),
        m_frontRight->GetState(),
        m_backLeft->GetState(),
        m_backRight->GetState()
    )};
    return speeds;
}

void SwerveDrive::DriveRobotRelative(frc::ChassisSpeeds speeds) {
     wpi::array<frc::SwerveModuleState, 4> states {kDriveKinematics.ToSwerveModuleStates(speeds)};
    //speeds = frc::ChassisSpeeds{5_mps, 0_mps, 0_rad_per_s};
    kDriveKinematics.DesaturateWheelSpeeds(&states, SwerveModule::kDriveMaxSpeed);
    m_frontLeft->SetDesiredState(states[0]);
    m_frontRight->SetDesiredState(states[1]);
    m_backLeft->SetDesiredState(states[2]);
    m_backRight->SetDesiredState(states[3]);

    }





// This method will be called once per scheduler run
void SwerveDrive::Periodic() {
    m_odo.Update(GetAngle(), GetPositions());
    
    using namespace frc;
    SmartDashboard::PutNumber("Swerve/Odo/Angle", GetAngle().Degrees().value());
    SmartDashboard::PutNumber("Swerve/Odo/PoseX", GetPose().X().value());
    SmartDashboard::PutNumber("Swerve/Odo/PoseY", GetPose().Y().value());

    SmartDashboard::PutNumber("Swerve/Odo/FrontLeftDistance", m_frontLeft->GetPosition().distance.value());
    SmartDashboard::PutNumber("Swerve/Odo/FrontRightDistance", m_frontRight->GetPosition().distance.value());
    SmartDashboard::PutNumber("Swerve/Odo/BackLeftDistance", m_backLeft->GetPosition().distance.value());
    SmartDashboard::PutNumber("Swerve/Odo/BackRightDistance", m_backRight->GetPosition().distance.value());

    SmartDashboard::PutNumber("Swerve/Odo/FrontLeftRotation", m_frontLeft->GetPosition().angle.Degrees().value());
    SmartDashboard::PutNumber("Swerve/Odo/FrontRightRotation", m_frontRight->GetPosition().angle.Degrees().value());
    SmartDashboard::PutNumber("Swerve/Odo/BackLeftRotation", m_backLeft->GetPosition().angle.Degrees().value());
    SmartDashboard::PutNumber("Swerve/Odo/BackRightRotation", m_backRight->GetPosition().angle.Degrees().value());


    SmartDashboard::PutData("Swerve/Odo/ResetHeading", resetHeadingCommand);
}
