/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  js1 = new frc::Joystick(0);

  l1 = new WPI_TalonSRX(1);
  l2 = new WPI_TalonSRX(4);

  r1 = new WPI_TalonSRX(2);
  r2 = new WPI_TalonSRX(6);

  yAxis = 0.0;
  rotAxis = 0.0;

  ahrs = new AHRS(SPI::Port::kMXP);

  turnController = new frc::PIDController(kP, kI, kD, kF, ahrs, this, 0.05);
  turnController->SetInputRange(-180.0f,  180.0f);
  turnController->SetOutputRange(-1.0, 1.0);
  turnController->SetAbsoluteTolerance(kToleranceDegrees);
  turnController->SetContinuous(true);
}

void Robot::DreadbotTankDrive() {
  yAxis = -js1->GetRawAxis(1);
  rotAxis = -js1->GetRawAxis(0);

  if(fabs(yAxis) < joystickDeadband) {
    yAxis = 0.0;
  }

  if(fabs(rotAxis) < joystickDeadband) {
    rotAxis = 0.0;
  }

  if(fabs(rotAxis) > 0.0) {
    rotSpeed = -rotSpeed;
  }

  ySpeed = yAxis * speed;
  rotSpeed = rotAxis * speed;

  lFinalSpeed = ySpeed + -rotSpeed;
  rFinalSpeed = ySpeed + rotSpeed;

  l1->Set(ControlMode::PercentOutput, lFinalSpeed);
  l2->Set(ControlMode::PercentOutput, lFinalSpeed);

  r1->Set(ControlMode::PercentOutput, -rFinalSpeed);
  r2->Set(ControlMode::PercentOutput, -rFinalSpeed);
}

void Robot::RotateToAngle(double targetAngle, double currentAngle){ //angle is -180 to 180
  bool rotateToAngle = false;
  if ( js1->GetRawButton(2)) {
    turnController->SetSetpoint(0.0f);
    rotateToAngle = true;
  } 
  else if ( js1->GetRawButton(3)) {
    turnController->SetSetpoint(90.0f);
    rotateToAngle = true;
  }
  else if ( js1->GetRawButton(4)) {
    turnController->SetSetpoint(179.9f);
    rotateToAngle = true;
  } 
  else if ( js1->GetRawButton(5)) {
    turnController->SetSetpoint(-90.0f);
    rotateToAngle = true;
  }

  double currentRotationRate;
  if ( rotateToAngle ) {
    turnController->Enable();
    currentRotationRate = rotateToAngleRate;
  } 
  else {
    turnController->Disable();
    currentRotationRate = js1->GetTwist();
  }
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  DreadbotTankDrive();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
