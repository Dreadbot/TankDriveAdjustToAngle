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
#include <frc/controller/PIDController.h>
#include <frc/PIDOutput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>

#include <frc/WPILib.h>

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
  void DreadbotTankDrive(double y_axis, double rot_axis, bool checkForDeadband);

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
  double kP = 0.1;
  double kI = 0.00;
  double kD = 0.00;
  double kF = 0.00;

  const double kPIDPeriod = 20.0;

  const double kToleranceDegrees = 2.0;

  const double kMinInputRange = -180.0;
  const double kMaxInputRange = 180.0;

  // PID VARIABLES
  double rotate_to_angle_rate = 0.0;
  double current_rotation_rate = 0.0;

  int timeToAdjust = 500;

  double error = 0.0;

  bool rotate_to_angle = false;

  // JOYSTICK INPUT OBJECTS
  frc::Joystick *joystick_1;

  // MOTOR OBJECTS
  WPI_TalonSRX *left_motor_1;
  WPI_TalonSRX *left_motor_2;

  WPI_TalonSRX *right_motor_1;
  WPI_TalonSRX *right_motor_2;

  // ROBOT GYRO OBJECTS/VARIABLES
  AHRS *ahrs;
  int BUTTON_TIMEOUT = 0;
  double slop = 3;
  bool turnComplete = false;
  double selectedAngle = 0;

  double const kCardinalDegrees[4] = {-90.0, 90.0, 179.9, 0.0};

  // PID OBJECTS
  frc2::PIDController *turn_controller;
  
  // TANK DRIVE VARIABLES
  double y_speed;
  double rot_speed;

  double left_final_speed;
  double right_final_speed;
};
