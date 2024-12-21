// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/JoystickDrive.h"

JoystickDrive::JoystickDrive(SwerveDrive* swerve, std::function<double()> xSpeedFunction, std::function<double()> ySpeedFunction, std::function<double()> turnSpeed) {
  //these between -1 and 1
  m_swerve = swerve;
  m_xSpeed = xSpeedFunction;
  m_ySpeed = ySpeedFunction;
  m_turnSpeed = turnSpeed;
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve);
}

// Called when the command is initially scheduled.
void JoystickDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void JoystickDrive::Execute() {
  xSpeed = units::velocity::meters_per_second_t{m_xSpeed() * 10};
  ySpeed = units::velocity::meters_per_second_t{m_ySpeed() * 10};
  turnSpeed = units::radians_per_second_t{m_turnSpeed() * std::numbers::pi};
  //xSpeed *= Math.signum(xSpeed) * xSpeed;
  //ySpeed *= Math.signum(ySpeed) * ySpeed;

  chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, m_swerve->GetAngle());
  m_swerve->DriveRobotRelative(chassisSpeeds);
}

// Called once the command ends or is interrupted.
void JoystickDrive::End(bool interrupted) {
  m_swerve->Stop();
}

// Returns true when the command should end.
bool JoystickDrive::IsFinished() {
  return false;
}
