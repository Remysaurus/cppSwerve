// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>
#include "subsystems/SwerveDrive.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class JoystickDrive
    : public frc2::CommandHelper<frc2::Command, JoystickDrive> {
 public:
  JoystickDrive(SwerveDrive* swerve, std::function<double()> xSpeedFunction, std::function<double()> ySpeedFunction, std::function<double()> turnSpeed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:

  SwerveDrive* m_swerve;
  std::function<double()> m_xSpeed;
  std::function<double()> m_ySpeed;
  std::function<double()> m_turnSpeed;

  units::velocity::meters_per_second_t xSpeed;
  units::velocity::meters_per_second_t ySpeed;
  units::radians_per_second_t turnSpeed;

  frc::ChassisSpeeds chassisSpeeds;

  //slew rate limiter needed



  
  



};
