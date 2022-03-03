// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <chrono>
#include <thread>

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
  float autoSpeed = 300;
  bool targeted = false;
  double targetOffsetAngle_Horizontal = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);;
  double targetArea = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta",0.0);
  double targetSkew = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts",0.0);
  double validTarget = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0);

  m_l1.SetNeutralMode(NeutralMode::Brake);
  m_l2.SetNeutralMode(NeutralMode::Brake);
  m_r2.SetNeutralMode(NeutralMode::Brake);
  m_r1.SetNeutralMode(NeutralMode::Brake);

  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",0);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",3);
  

  if((nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0) != 1)){
    m_l1.Set(-.2);
    m_l2.Set(-.2);

    m_r1.Set(-.2);
    m_r2.Set(-.2);
  }else{
    if(nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0) < -10){
      m_l1.Set(-.2);
      m_l2.Set(-.2);

      m_r1.Set(-.2);
      m_r2.Set(-.2);
    }else if(nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0) > 10){
      m_l1.Set(.1);
      m_l2.Set(.1);

      m_r1.Set(.1);
      m_r2.Set(.1);
    }else{
      m_l1.Set(0);
      m_l2.Set(0);

      m_r1.Set(0);
      m_r2.Set(0);
    }
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
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    swap = currentState;
    currentState = nextState;
    nextState = swap;
    frc::SmartDashboard::PutNumber("SongState", currentState);
  }
  if(currentState == 0){
  Drive(m_driverController.GetLeftY(),m_driverController.GetRightY(), m_driverController.GetRightBumper());
  }else{
    if(m_driverController.GetBButton()==1){
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      if(index < 13){
      index++;
      }else if(index == 13){
        index = 0;
      }
      orchestra.Stop();
      orchestra.LoadMusic(_songs[index]);
      orchestra.Play();
    }
  }
  stack(m_driverController2.GetAButton(),m_driverController2.GetBButton());
  arm(m_driverController2.GetXButton(),m_driverController2.GetYButton());
  climber(m_driverController.GetYButton(),m_driverController.GetAButton());
  shoot(m_driverController2.GetRightTriggerAxis(),m_driverController2.GetLeftTriggerAxis());

  if(m_driverController2.GetBackButton() == 1){
    upSolenoid.Set(true);
    downSolenoid.Set(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    upSolenoid.Set(false);
    downSolenoid.Set(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    upSolenoid.Set(false);
    downSolenoid.Set(false);
  }

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

  if(state == 1){
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    swap2 = currentState2;
    currentState2 = nextState2;
    nextState2 = swap2;
    frc::SmartDashboard::PutNumber("DriveState", currentState2);
  }

  if(currentState2 == 0){
    float fullSpeedPer = .4;
    m_l1.Set(-fullSpeedPer * left);
    m_l2.Set(-fullSpeedPer * left);

    m_r1.Set(fullSpeedPer * right);
    m_r2.Set(fullSpeedPer * right);
  } else if(currentState2 == 1){
    float perSpeed = .2;
    m_l1.Set(-perSpeed * left);
    m_l2.Set(-perSpeed * left);

    m_r1.Set(perSpeed * right);
    m_r2.Set(perSpeed * right);
  }
}

void Robot::climber(int up, int down){
  m_climber.SetNeutralMode(NeutralMode::Brake);
  if(up==1){
    m_climber.Set(.4);
  }else if(down==1){
    m_climber.Set(-.4);
  }else{
    m_climber.Set(0);
  }
}


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

void Robot::shoot(float speed1, float speed2){
  if(speed1 > .7){
    m_shooter.Set(1);
  }else if(speed2 > .7){
    m_shooter.Set(.5);
  }else{
    m_shooter.Set(0);
  }

}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
