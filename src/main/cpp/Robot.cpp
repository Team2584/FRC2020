/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/encoder.h>
#include "ctre/Phoenix.h"
#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include "frc/WPILib.h"
#include <stdio.h>
#include <memory>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <math.h>

using namespace frc;
/*
888b     d888          888                          .d8888b.           888                      
8888b   d8888          888                         d88P  Y88b          888                      
88888b.d88888          888                         Y88b.               888                      
888Y88888P888  .d88b.  888888 .d88b.  888d888       "Y888b.    .d88b.  888888 888  888 88888b.  
888 Y888P 888 d88""88b 888   d88""88b 888P"            "Y88b. d8P  Y8b 888    888  888 888 "88b 
888  Y8P  888 888  888 888   888  888 888                "888 88888888 888    888  888 888  888 
888   "   888 Y88..88P Y88b. Y88..88P 888          Y88b  d88P Y8b.     Y88b.  Y88b 888 888 d88P 
888       888  "Y88P"   "Y888 "Y88P"  888           "Y8888P"   "Y8888   "Y888  "Y88888 88888P"  
                                                                                       888      
                                                                                       888      
                                                                                       888      
*/

static const int leftleadmotorID = 2, rightleadmotorID = 4, leftfollowmotorID = 3 , rightfollowermotorID = 5;
  rev::CANSparkMax m_leftleadmotor{leftleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftfollowermotor{leftfollowmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightleadmotor{rightleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightfollowermotor{rightfollowermotorID, rev::CANSparkMax::MotorType::kBrushless};
  

  frc::DifferentialDrive m_robotDrive{m_leftleadmotor, m_rightleadmotor};
  
  frc::Joystick *m_stick;
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


  m_stick = new Joystick(0);
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

void Robot::TeleopInit() {
  m_leftfollowermotor.Follow(m_leftleadmotor);
  m_rightfollowermotor.Follow(m_rightleadmotor);
}

void Robot::TeleopPeriodic() {
  m_robotDrive.ArcadeDrive(-m_stick->GetY(), m_stick->GetZ());
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
