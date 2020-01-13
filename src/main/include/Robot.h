/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#ifndef DREADBOTTANKDRIVE_INCLUDE_ROBOT_H_
#define DREADBOTTANKDRIVE_INCLUDE_ROBOT_H_

#pragma once

#include <iostream>
#include <string>

#include <AHRS.h>

#include <ctre/Phoenix.h>

#include <frc/Joystick.h>
#include <frc/PIDController.h>
#include <frc/PIDOutput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>

#include "RobotUtilities.h"

#endif // DREADBOTTANKDRIVE_INCLUDE_ROBOT_H_

class Robot : public frc::TimedRobot, public PIDOutput {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void PIDWrite(double output) {
    rotateToAngleRate = output;
  }

  void RotateToAngle(double targetAngle, double currentAngle);
  void DreadbotTankDrive(double yAxis, double rotAxis);

private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  const double joystickDeadband = 0.2;
  const double speed = 0.4;

  const double kP = 0.03f;
  const double kI = 0.00f;
  const double kD = 0.00f;
  const double kF = 0.00f;
  const double kToleranceDegrees = 2.0f;

  double rotateToAngleRate = 0.0;

  // JOYSTICK INPUTS
  frc::Joystick *js1;

  // MOTORS
  WPI_TalonSRX *l1;
  WPI_TalonSRX *l2;

  WPI_TalonSRX *r1;
  WPI_TalonSRX *r2;

  AHRS *ahrs; 
  frc::PIDController *turnController;
  
  double ySpeed;
  double rotSpeed;

  double lFinalSpeed;
  double rFinalSpeed;
};
