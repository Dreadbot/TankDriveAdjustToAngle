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
  joystick_1 = new frc::Joystick(0);

  // Robot Motor Objects
  left_motor_1 = new WPI_TalonSRX(1);
  left_motor_2 = new WPI_TalonSRX(4);

  right_motor_1 = new WPI_TalonSRX(2);
  right_motor_2 = new WPI_TalonSRX(6);

  // Robot Gyroscope Objects
  ahrs = new AHRS(SPI::Port::kMXP);

  // PID Controller Object Setup
  turn_controller = new frc::PIDController(kP, kI, kD, kF, ahrs, this, 0.05);
  turn_controller->SetInputRange(-180.0f,  180.0f);
  turn_controller->SetOutputRange(-1.0, 1.0);
  turn_controller->SetAbsoluteTolerance(kToleranceDegrees);
  turn_controller->SetContinuous(true);
}

void Robot::DreadbotTankDrive(double yAxis, double rotAxis) {
  // Account for Joystick Deadband
  if(fabs(yAxis) < kJoystickDeadband) {
    yAxis = 0.0;
  }

  if(fabs(rotAxis) < kJoystickDeadband) {
    rotAxis = 0.0;
  }

  // Flipping Variables to change behavior for rotation.
  if(fabs(rotAxis) > 0.0) {
    rot_speed = -rot_speed;
  }

  // Multiply Intended Velocity by a cap speed.
  // (See main.include.Robot.h)
  y_speed = yAxis * kSpeed;
  rot_speed = rotAxis * kSpeed;

  // Calculating Final Speed
  // by Adding the Rotation Factor
  left_final_speed = y_speed + -rot_speed;
  right_final_speed = y_speed + rot_speed;

  // Set Motor Values
  left_motor_1->Set(ControlMode::PercentOutput, left_final_speed);
  left_motor_2->Set(ControlMode::PercentOutput, left_final_speed);

  right_motor_1->Set(ControlMode::PercentOutput, -right_final_speed);
  right_motor_2->Set(ControlMode::PercentOutput, -right_final_speed);
}

void Robot::RotateToAngle(double targetAngle, double currentAngle){ //angle is -180 to 180
  // Code by KauaiLabs for Angle Rotation using Gyro.
  // https://pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/
  
  bool rotateToAngle = false;
  if ( joystick_1->GetRawButton(2)) {
    turn_controller->SetSetpoint(kCardinalDegrees[0]);
    rotateToAngle = true;
  } 
  else if ( joystick_1->GetRawButton(3)) {
    turn_controller->SetSetpoint(kCardinalDegrees[1]);
    rotateToAngle = true;
  }
  else if ( joystick_1->GetRawButton(4)) {
    turn_controller->SetSetpoint(kCardinalDegrees[2]);
    rotateToAngle = true;
  } 
  else if ( joystick_1->GetRawButton(5)) {
    turn_controller->SetSetpoint(kCardinalDegrees[3]);
    rotateToAngle = true;
  }

  double currentRotationRate;
  if ( rotateToAngle ) {
    turn_controller->Enable();
    currentRotationRate = rotate_to_angle_rate;
  } 
  else {
    turn_controller->Disable();
    currentRotationRate = joystick_1->GetTwist();
  }

  // Rotate to angle using TankDrive function.
  DreadbotTankDrive(-joystick_1->GetRawAxis(y_axis), currentRotationRate);
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
  DreadbotTankDrive(-joystick_1->GetRawAxis(y_axis), -joystick_1->GetRawAxis(x_axis));

  // Check buttons to rotate to angle.
  // Inherently Overrides DreadBotTankDrive function.
  if(joystick_1->GetRawButton(a_button)) {
    RotateToAngle(kCardinalDegrees[0], ahrs->GetAngle());
  } else if(joystick_1->GetRawButton(b_button)) {
    RotateToAngle(kCardinalDegrees[1], ahrs->GetAngle());
  } else if(joystick_1->GetRawButton(y_button)) {
    RotateToAngle(kCardinalDegrees[2], ahrs->GetAngle());
  } else if(joystick_1->GetRawButton(x_button)) {
    RotateToAngle(kCardinalDegrees[3], ahrs->GetAngle());
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif