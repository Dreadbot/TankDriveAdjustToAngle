/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit() {
  // Setup for SmartDashboard.
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Input Device Objects
  js1 = new frc::Joystick(0);

  // Robot Motor Objects
  l1 = new WPI_TalonSRX(1);
  l2 = new WPI_TalonSRX(4);

  r1 = new WPI_TalonSRX(2);
  r2 = new WPI_TalonSRX(6);

  // Robot Gyroscope Objects
  ahrs = new AHRS(SPI::Port::kMXP);

  // PID Controller Object Setup
  turnController = new frc::PIDController(kP, kI, kD, kF, ahrs, this, 0.05);
  turnController->SetInputRange(-180.0f,  180.0f);
  turnController->SetOutputRange(-1.0, 1.0);
  turnController->SetAbsoluteTolerance(kToleranceDegrees);
  turnController->SetContinuous(true);
}

void Robot::DreadbotTankDrive(double yAxis, double rotAxis) {
  // Account for Joystick Deadband
  if(fabs(yAxis) < joystickDeadband) {
    yAxis = 0.0;
  }

  if(fabs(rotAxis) < joystickDeadband) {
    rotAxis = 0.0;
  }

  // Flipping Variables to change behavior for rotation.
  if(fabs(rotAxis) > 0.0) {
    rotSpeed = -rotSpeed;
  }

  // Multiply Intended Velocity by a cap speed.
  // (See main.include.Robot.h)
  ySpeed = yAxis * speed;
  rotSpeed = rotAxis * speed;

  // Calculating Final Speed
  // by Adding the Rotation Factor
  lFinalSpeed = ySpeed + -rotSpeed;
  rFinalSpeed = ySpeed + rotSpeed;

  // Set Motor Values
  l1->Set(ControlMode::PercentOutput, lFinalSpeed);
  l2->Set(ControlMode::PercentOutput, lFinalSpeed);

  r1->Set(ControlMode::PercentOutput, -rFinalSpeed);
  r2->Set(ControlMode::PercentOutput, -rFinalSpeed);
}

void Robot::RotateToAngle(double targetAngle, double currentAngle){ //angle is -180 to 180
  // Code by KauaiLabs for Angle Rotation using Gyro.
  // https://pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/
  
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

  // Rotate to angle using TankDrive function.
  DreadbotTankDrive(-js1->GetRawAxis(y_axis), currentRotationRate);
}

void Robot::RobotPeriodic() {}

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
  DreadbotTankDrive(-js1->GetRawAxis(y_axis), -js1->GetRawAxis(x_axis));

  // Check buttons to rotate to angle.
  // Inherently Overrides DreadBotTankDrive function.
  if(js1->GetRawButton(a_button)) {
    RotateToAngle(0.0, ahrs->GetAngle());
  } else if(js1->GetRawButton(b_button)) {
    RotateToAngle(90.0, ahrs->GetAngle());
  } else if(js1->GetRawButton(y_button)) {
    RotateToAngle(180.0, ahrs->GetAngle());
  } else if(js1->GetRawButton(x_button)) {
    RotateToAngle(-90.0, ahrs->GetAngle());
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif