// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <iostream>
#include <frc/Timer.h>
#include <string>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include <chrono>
#include <thread>
#include <math.h>
#include <cameraserver/CameraServer.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;
  //written to control left side and right side independantly with one line
  // Drive(Left side speed (-1 to 1), right side speed (-1 to 1))
  void Drive(float left, float right, int state);
  void stack(int up, int down);
  void climber(int up, int down);
  void arm(int in, int out);
  void shoot(float speed1, float speed2, int reverse);
  void getDistance();
  void autoDrive(float left, float right);
  void rangeFind(float less, float more);

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  private:
  //Declaring xbox contoller
  Orchestra orchestra;
  frc::XboxController m_driverController{0};
  frc::XboxController m_driverController2{1};
  //Declaring motors and establishing CAN IDs 
  //use phoenix tuner to get CAN IDs and put them in the brackets
  WPI_TalonFX m_r1{4};
  WPI_TalonFX m_r2{3};

  WPI_TalonSRX m_stack{7};
  WPI_TalonSRX m_arm{13};
  WPI_TalonSRX m_shooter{15};

  WPI_TalonFX m_l1{1};
  WPI_TalonFX m_l2{2};
  
  WPI_TalonFX m_climber{5};
  
  frc::Solenoid downSolenoid{frc::PneumaticsModuleType::CTREPCM, 0};
  frc::Solenoid upSolenoid{frc::PneumaticsModuleType::CTREPCM, 1};

  //Declaring timer to use for timed events like autonomous
  frc::Timer timer;
  int index = 0;
  int currentState = 0;
  int nextState = 1;
  int swap;

  bool load = false; 
  bool fired = false;
  bool stage0 = false;
  bool stage1 = false;
  bool stage1_5 = false;
  bool stage2 = false;
  bool stage3 = false;
  bool stage4 = false;

  int currentState2 = 0;
  int nextState2 = 1;
  int swap2;

  float currLeft = 0;
  float currRight = 0;

  double HeightToTarget = 103;
  double HeightToCamera = 24.5;
  double CameraAngle = 48.85;
  double DistanceToBumper = 12;
  double distance;
  double goalAngleDEG;
  double goalAngleRAD;

  std::string _songs[14] = {"tokyo.chrp","roll4.chrp","cantina.chrp","stand.chrp","iran.chrp","cruel.chrp","drift.chrp","march3.chrp","mega.chrp","roll1.chrp","shrek.chrp","tom.chrp", "ussr.chrp","wii.chrp",};
};
