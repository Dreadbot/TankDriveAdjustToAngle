/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>

#include <ctre/Phoenix.h>

#include "AHRS.h"
#include <frc/PIDController.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  void RotateToAngle(int targetAngle);
  void DreadbotTankDrive();

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

  // JOYSTICK INPUTS
  frc::Joystick *js1;

  // MOTORS
  WPI_TalonSRX *l1;
  WPI_TalonSRX *l2;

  WPI_TalonSRX *r1;
  WPI_TalonSRX *r2;

  AHRS *ahrs; 
  PIDController *turnController;

  double yAxis;
  double rotAxis;
  
  double ySpeed;
  double rotSpeed;

  double lFinalSpeed;
  double rFinalSpeed;
};
