// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  timer.Reset();
  timer.Start();
}

void Robot::AutonomousPeriodic() {
  if(timer.Get() <  1_s){
    m_l1.Set(-1);
    m_l2.Set(-1);

    m_r1.Set(1);
    m_r2.Set(1);
  }else{
    m_l1.Set(0);
    m_l2.Set(0);

    m_r1.Set(0);
    m_r2.Set(0);
  }

  TalonFX * talon = new TalonFX(1);
  talon -> ConfigSelectedFeedbackSensor(FeedbackDevice::Tachometer, 0, 0); /* PIDLoop=0, timeoutMs=0 */
  talon -> Set(ControlMode::Velocity, 600);
}

void Robot::TeleopInit() {
}
void Robot::TeleopPeriodic() {
  //calling drive with xbox stick input
  Drive(-m_driverController.GetRightY(),m_driverController.GetRightY(), m_driverController.GetRightBumper());
  //Song(m_driverController.GetAButton());
}

void Robot::DisabledInit() {
  Drive(0,0,0);
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::Drive(float left, float right, int state){
  //setting motor speeds to parameters
  if(state == 0){
    m_l1.Set(left);
    m_l2.Set(left);

    m_r1.Set(right);
    m_r2.Set(right);
} else if(state == 1){
    m_l1.Set(.2*left);
    m_l2.Set(.2*left);

    m_r1.Set(.2*right);
    m_r2.Set(.2*right);
  }
}
/*
void Robot::Song(int button){
  if(load == false){
    orchestra.AddInstrument(m_l1);
    orchestra.AddInstrument(m_l2);
    orchestra.AddInstrument(m_r1);
    orchestra.AddInstrument(m_r2);
    orchestra.LoadMusic("roll.chrp");
    load = true;
  }
  if(button == 1){
    orchestra.Play();
  }else{
    orchestra.Pause();
  }
}*/

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
