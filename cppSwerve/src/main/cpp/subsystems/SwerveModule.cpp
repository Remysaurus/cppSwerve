// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>

SwerveModule::SwerveModule(
    int driveMotorID, 
    int turnMotorID, 
    int turnEncoderPort, 
    double turnEncoderOffset, 
    bool driveInverted, 
    bool turnInverted) {
        m_driveMotor = new ctre::phoenix6::hardware::TalonFX{driveMotorID};
        ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
        driveConfig.MotorOutput.Inverted = driveInverted;
        driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        driveConfig.Feedback.SensorToMechanismRatio = kDriveDistancePerEncoderRotation;
        m_driveMotor->GetConfigurator().Apply(driveConfig);

        m_turnMotor = new rev::CANSparkMax{turnMotorID, rev::CANSparkLowLevel::MotorType::kBrushless};
        m_turnMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_turnMotor->SetInverted(turnInverted);
        
        m_turnEncoder = new frc::DutyCycleEncoder{turnEncoderPort};
        m_turnEncoder->SetDistancePerRotation(2 * std::numbers::pi);
        m_turnEncoder->SetPositionOffset(turnEncoderOffset);

        m_drivePIDController = new frc::PIDController{kPDrive, kIDrive, kDDrive};
        m_turnPIDController = new frc::PIDController{kPTurn, kITurn, kDTurn};
        m_turnPIDController->EnableContinuousInput(-std::numbers::pi, +std::numbers::pi);

    }

units::length::meter_t SwerveModule::GetDrivePosition() {
    units::meter_t position{m_driveMotor->GetPosition().GetValueAsDouble()};
    return position;
}

units::velocity::meters_per_second_t SwerveModule::GetDriveVelocity() {
    units::meters_per_second_t velocity{m_driveMotor->GetVelocity().GetValueAsDouble()};
    return velocity;
}

frc::Rotation2d SwerveModule::GetTurnPosition() {
    units::radian_t radians{m_turnEncoder->GetDistance()};
    frc::Rotation2d rotation(radians);
    return rotation;
}

void SwerveModule::ResetEncoders() {
    m_driveMotor->SetPosition(0.0_rad);
}

frc::SwerveModuleState SwerveModule::GetState() {
    frc::SwerveModuleState state{GetDriveVelocity(), GetTurnPosition()};
    return state;
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
    frc::SwerveModulePosition position{GetDrivePosition(), GetTurnPosition()};
    return position;
}

void SwerveModule::Stop() {
    m_driveMotor->Set(0.0);
    m_turnMotor->Set(0.0);
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState state) {
    if (abs(state.speed.value()) < kDriveDeadband) {
        Stop();
        return;
    }
    state = state.Optimize(state, state.angle);

    double ff = (state.speed.value())/kDriveMaxSpeed.value();
    double pid = m_drivePIDController->Calculate((GetDriveVelocity().value()), state.speed.value());
    
    m_driveMotor->Set(ff + pid);
    //m_driveMotor->Set(0.25);
    m_turnMotor->Set(m_turnPIDController->Calculate(((GetTurnPosition().Radians().value()), (state.angle.Radians().value()))));
    //m_turnMotor->Set(m_turnPIDController->Calculate(((GetTurnPosition().Radians().value()), 0)));
}



// This method will be called once per scheduler run
void SwerveModule::Periodic() {  
}
