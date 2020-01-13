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
    rotate_to_angle_rate = output;
  }

  void RotateToAngle(double target_angle, double current_angle);
  void DreadbotTankDrive(double y_axis, double rot_axis);

private:
  // SMARTDASHBOARD VARIABLES/CONSTANTS
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // TANK DRIVE CONSTANTS
  const double kJoystickDeadband = 0.2;
  const double kSpeed = 0.4;

  // PID CONSTANTS
  const double kP = 0.03f;
  const double kI = 0.00f;
  const double kD = 0.00f;
  const double kF = 0.00f;
  const double kToleranceDegrees = 2.0f;

  // PID VARIABLES
  double rotate_to_angle_rate = 0.0;

  // JOYSTICK INPUT OBJECTS
  frc::Joystick *joystick_1;

  // MOTOR OBJECTS
  WPI_TalonSRX *left_motor_1;
  WPI_TalonSRX *left_motor_2;

  WPI_TalonSRX *right_motor_1;
  WPI_TalonSRX *right_motor_2;

  // ROBOT GYRO OBJECTS/VARIABLES
  AHRS *ahrs;

  double const kCardinalDegrees[4] = {0.0, 90.0, 179.9, -90.0};

  // PID OBJECTS
  frc::PIDController *turn_controller;
  
  // TANK DRIVE VARIABLES
  double y_speed;
  double rot_speed;

  double left_final_speed;
  double right_final_speed;
};
