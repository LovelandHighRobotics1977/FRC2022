// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <unistd.h>

void Robot::RobotInit() {
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double kF = 1; //Feed Forward
  double gRef = 0; //Setpoint

  m_l1.ConfigFactoryDefault();
  m_l1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0); // PIDLoop=0, timeoutMs=0 
  m_l2.ConfigFactoryDefault();
  m_l2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0); // PIDLoop=0, timeoutMs=0
  m_r1.ConfigFactoryDefault();
  m_r1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0); // PIDLoop=0, timeoutMs=0 
  m_r2.ConfigFactoryDefault();
  m_r2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0); // PIDLoop=0, timeoutMs=0

  m_l1.Config_kP(0, kP);
  m_l1.Config_kI(0, kI);
  m_l1.Config_kD(0, kD);
  m_l1.Config_kF(0, kF);

  m_l2.Config_kP(0, kP);
  m_l2.Config_kI(0, kI);
  m_l2.Config_kD(0, kD);
  m_l2.Config_kF(0, kF);

  m_r1.Config_kP(0, kP);
  m_r1.Config_kI(0, kI);
  m_r1.Config_kD(0, kD);
  m_r1.Config_kF(0, kF);

  m_r2.Config_kP(0, kP);
  m_r2.Config_kI(0, kI);
  m_r2.Config_kD(0, kD);
  m_r2.Config_kF(0, kF);

  m_l1.ConfigNominalOutputForward(0);
	m_l1.ConfigNominalOutputReverse(0);
	m_l1.ConfigPeakOutputForward(1);
	m_l1.ConfigPeakOutputReverse(-1);

  m_l2.ConfigNominalOutputForward(0);
	m_l2.ConfigNominalOutputReverse(0);
	m_l2.ConfigPeakOutputForward(1);
	m_l2.ConfigPeakOutputReverse(-1);

  m_r1.ConfigNominalOutputForward(0);
	m_r1.ConfigNominalOutputReverse(0);
	m_r1.ConfigPeakOutputForward(1);
	m_r1.ConfigPeakOutputReverse(-1);

  m_r2.ConfigNominalOutputForward(0);
	m_r2.ConfigNominalOutputReverse(0);
	m_r2.ConfigPeakOutputForward(1);
	m_r2.ConfigPeakOutputReverse(-1);

  timer.Reset();
  timer.Start();
  
}

void Robot::AutonomousPeriodic() {
  m_l1.SetNeutralMode(NeutralMode::Brake);
  m_l2.SetNeutralMode(NeutralMode::Brake);
  m_r2.SetNeutralMode(NeutralMode::Brake);
  m_r1.SetNeutralMode(NeutralMode::Brake);
  if(timer.Get() < 1_s){
    m_l1.Set(ControlMode::Velocity, 300);
    m_l2.Set(ControlMode::Velocity, 300);
    m_r1.Set(ControlMode::Velocity, -300);
    m_r2.Set(ControlMode::Velocity, -300);

  }else{
    m_l1.Set(0);
    m_l2.Set(0);
    m_r1.Set(0);
    m_r2.Set(0);
  }
  
}

void Robot::TeleopInit() {
  orchestra.AddInstrument(m_l1);
  orchestra.AddInstrument(m_l2);
  orchestra.AddInstrument(m_r1);
  orchestra.AddInstrument(m_r2);
}
void Robot::TeleopPeriodic() {

  //calling drive with xbox stick input
  if(m_driverController.GetStartButton() == 1){
    swap = currentState;
    currentState = nextState;
    nextState = swap;
  }
  //Song(m_driverController.GetLeftStickButton(),m_driverController.GetLeftBumper());
  if(currentState == 0){
  Drive(.5*-m_driverController.GetLeftY(),.5*m_driverController.GetRightY(), m_driverController.GetRightBumper());
  }else{
    if(m_driverController.GetBButton()==1){
      sleep(2);
      if(index < 14){
      index++;
      }else if(index >= 14){
        index = 0;
      }
      orchestra.Stop();
      orchestra.LoadMusic(_songs[index]);
      orchestra.Play();
    }
  }
  stack(m_driverController2.GetAButton(),m_driverController2.GetBButton());
  arm(m_driverController2.GetXButton(),m_driverController2.GetYButton());

  if((m_driverController2.GetLeftBumper() == 1)&&(m_driverController2.GetRightBumper() == 0)){
    upSolenoid.Set(true);
    downSolenoid.Set(false);
  }else if((m_driverController2.GetRightBumper() == 1)&&(m_driverController2.GetLeftBumper() == 0)){
    upSolenoid.Set(false);
    downSolenoid.Set(true);
  }else{
    upSolenoid.Set(false);
    downSolenoid.Set(false);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::Drive(float left, float right, int state){
  //setting motor speeds to parameters
  m_l1.SetNeutralMode(NeutralMode::Coast);
  m_l2.SetNeutralMode(NeutralMode::Coast);
  m_r2.SetNeutralMode(NeutralMode::Coast);
  m_r1.SetNeutralMode(NeutralMode::Coast);
  if(state == 0){
    m_l1.Set(left);
    m_l2.Set(left);

    m_r1.Set(right);
    m_r2.Set(right);
} else if(state == 1){
    m_l1.Set(.3*left);
    m_l2.Set(.3*left);

    m_r1.Set(.3*right);
    m_r2.Set(.3*right);
  }
}

/*void climber(){
  gun.SetNeutralMode(NeutralMode::Brake);
}
*/

void Robot::stack(int up, int down){
  if((up==1) && (down==0)){
    m_stack.Set(1);
  }else if((up==0) && (down==1)){
    m_stack.Set(-1);
  }else{
    m_stack.Set(0);
  }
}

void Robot::arm(int in, int out){
  if((in==1) && (out==0)){
    m_arm.Set(1);
  }else if((in==0) && (out==1)){
    m_arm.Set(-1);
  }else{
    m_arm.Set(0);
  }
}


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
