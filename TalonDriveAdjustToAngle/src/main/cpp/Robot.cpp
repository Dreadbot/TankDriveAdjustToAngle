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
  ahrs->ZeroYaw();

  // PID Controller Object Setup
  turn_controller = new frc2::PIDController(kP, kI, kD);
  turn_controller->EnableContinuousInput(kMinInputRange, kMaxInputRange);

  frc::SmartDashboard::PutNumber("P value", kP);
  frc::SmartDashboard::PutNumber("I value", kI);
  frc::SmartDashboard::PutNumber("D value", kD);
}

void Robot::DreadbotTankDrive(double yAxis, double rotationAxis, bool checkForDeadband) {
  // Account for Joystick Deadband
  if(checkForDeadband) {
    yAxis = (fabs(yAxis) < kJoystickDeadband) ? 0 : yAxis;
    rotationAxis = (fabs(rotationAxis) < kJoystickDeadband) ? 0 : rotationAxis;
  }

  // Flipping Variables to change behavior for rotation influence.
  rot_speed = (fabs(rotationAxis) > 0.0) ? -rot_speed : rot_speed;

  // Multiply Intended Velocity by a cap speed.
  // (See main.include.Robot.h)
  y_speed = yAxis * kSpeed;
  rot_speed = rotationAxis * kSpeed;
  std::cout<<"yAxis: "<<yAxis<<" rotAxis: "<<rotationAxis<<std::endl;
  std::cout<<"y_speed: "<<y_speed<<" rot_speed: "<<rot_speed<<std::endl;
  // Calculating Final Speed
  // by Adding the Rotation Factor
  left_final_speed = scaleSpeed(y_speed -rot_speed);
  right_final_speed = scaleSpeed(y_speed + rot_speed);

  // Set Motor Values
  std::cout << "left_final_speed: " << left_final_speed << " right_final_speed: " << right_final_speed << std::endl;
  left_motor_1->Set(ControlMode::PercentOutput, left_final_speed);
  left_motor_2->Set(ControlMode::PercentOutput, left_final_speed);

  right_motor_1->Set(ControlMode::PercentOutput, -right_final_speed);
  right_motor_2->Set(ControlMode::PercentOutput, -right_final_speed);
}

int Robot::scaleSpeed(int speed) {
  if (speed > 1|| speed < -1) {
    return speed / abs(speed);
  }
}

void Robot::RotateToAngle(double targetAngle, double currentAngle){ //angle is -180 to 180
  
  // turn_controller->SetSetpoint(targetAngle);
  // turn_controller->Enable();
  // current_rotation_rate = rotate_to_angle_rate;
  double localMinSpeed = minRotationRate;
  double error = currentAngle - targetAngle;
  
  // If we are close, then start the timer so it stops
  if (fabs(error) < slop){
    BUTTON_TIMEOUT++;
    // localMinSpeed = 0.25;
  }

  //current_rotation_rate = turn_controller->Calculate(error);
  current_rotation_rate = (error * kP);
  // add min speed, so it always have enough power to turn
  if(current_rotation_rate < 0){
    current_rotation_rate -= localMinSpeed;
  }
  else if (current_rotation_rate > 0){
    current_rotation_rate += localMinSpeed;
  }
  // if(error < 0){
  //   current_rotation_rate = current_rotation_rate * -1;
  // }
  
  // continue adjusting until we are really close and it has
  // been adjusting for timeToAdjust iterations
  if(fabs(error) < slop && BUTTON_TIMEOUT > timeToAdjust){
    turnComplete = true;
    ahrs->ZeroYaw();
  }

  frc::SmartDashboard::PutNumber("rotation_rate", current_rotation_rate);
  std::cout << "current_rotation rate: " << current_rotation_rate << std::endl;
  // Rotate to angle using TankDrive function.
  DreadbotTankDrive(-joystick_1->GetRawAxis(y_axis), current_rotation_rate, false);   
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

void Robot::TeleopInit() {
  double selectedAngle = 0;
  ahrs->ZeroYaw();
}

void Robot::TeleopPeriodic() {
  frc::SmartDashboard::PutNumber("Current Angle", ahrs->GetAngle());
  
  kP = frc::SmartDashboard::GetNumber("P value", 0.00);
  kI = frc::SmartDashboard::GetNumber("I value", 0.00);
  kD = frc::SmartDashboard::GetNumber("D value", 0.00);
  turn_controller->SetP(kP);
  turn_controller->SetI(kI);
  turn_controller->SetD(kD);
  frc::SmartDashboard::PutNumber("P", turn_controller->GetP());
  frc::SmartDashboard::PutNumber("D", turn_controller->GetD());
  frc::SmartDashboard::PutNumber("rotate to angle rate", rotate_to_angle_rate);
  std::cout<<"y joystick Axis: "<<-joystick_1->GetRawAxis(y_axis)<<" joystick x Axis: "<<-joystick_1->GetRawAxis(x_axis)<<std::endl;
  
  DreadbotTankDrive(-joystick_1->GetRawAxis(y_axis), -joystick_1->GetRawAxis(x_axis), true);

  // Check buttons to rotate to angle.
  // Inherently Overrides DreadBotTankDrive function.

  if(joystick_1->GetRawButton(x_button)){
    std::cout << "x button pressed" << std::endl;
    BUTTON_TIMEOUT = 0;
    turnComplete = false;
    selectedAngle = kCardinalDegrees[0]; 
  }
  else if(joystick_1->GetRawButton(b_button)){
    BUTTON_TIMEOUT = 0;
    turnComplete = false;
    selectedAngle = kCardinalDegrees[1];
  }
  else if(joystick_1->GetRawButton(a_button)){
    BUTTON_TIMEOUT = 0;
    turnComplete = false;
    selectedAngle = kCardinalDegrees[2];
  }
  else if (joystick_1->GetRawButton(y_button)){
    BUTTON_TIMEOUT = 0;
    turnComplete = false;
    selectedAngle = kCardinalDegrees[3];
  }
  if (!turnComplete){
    RotateToAngle(selectedAngle, ahrs->GetAngle()); 
  }
  //std::cout << "error:" << error << std::endl;
  //std::cout << "BUTTON_TIMEOUT: " << BUTTON_TIMEOUT << "Turn Complete: " << turnComplete << "selectedAngle: " << selectedAngle << "currentAngle: " << ahrs->GetAngle() << std::endl;
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif