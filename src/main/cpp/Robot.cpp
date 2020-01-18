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
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"


using namespace frc;


double speedv = 0.0, wristpos = 0.0, errorv = 0.0, interv = 0.0, derav = 0.0, preverrorv = 0.0;
double kP = 0.3325, kI = 0.00075, kD = 0.016, kIz = 0.5, kFF = 0, kMaxOutput = 0.25, kMinOutput = -0.25, kMaxOutputL = 0.25, kMinOutputL = -0.25; 
int state = 0;


double leftleadmotorID = 4, rightleadmotorID = 2, leftfollowmotorID = 5 , rightfollowermotorID = 3;
  rev::CANSparkMax m_leftleadmotor{leftleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftfollowermotor{leftfollowmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightleadmotor{rightleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightfollowermotor{rightfollowermotorID, rev::CANSparkMax::MotorType::kBrushless};

  frc::Joystick *m_stick;
  frc::Joystick *m_stick2;

  TalonSRX *TurretTest;
  TalonSRX *Topfly;
  TalonSRX *Botfly;
  VictorSPX *Indexer;
  VictorSPX *Intake1;
  VictorSPX *Intake2;
std::shared_ptr<NetworkTable> table;

frc::DifferentialDrive m_robotDrive{m_leftleadmotor, m_rightleadmotor};

void Robot::RobotInit() {
  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  Ahorz = 0;
  Avert = 0;
   double tA = 0, tS = 0;
  //TurretTest = new TalonSRX(6);
  //Topfly = new TalonSRX(3);
  //Botfly = new TalonSRX(7);

  Indexer = new VictorSPX(4);

  Intake1 = new VictorSPX(1);
  //Intake2 = new VictorSPX(2);

  m_stick = new Joystick(0);
  m_stick2 = new Joystick(1);


SmartDashboard::PutNumber("horizontal", 0);
SmartDashboard::PutNumber("nums", 0);
SmartDashboard::PutNumber("top", 1);
SmartDashboard::PutNumber("bottom",1);
SmartDashboard::PutNumber("P", kP);
SmartDashboard::PutNumber("I", kI);
SmartDashboard::PutNumber("D", kD); 
SmartDashboard::PutNumber("*Speed", kMaxOutput); 
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
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  Ahorz = table->GetNumber("tx",0.0);
  Avert = table->GetNumber("ty",0.0);
  double tA = table->GetNumber("ta",0.0);
  double tS = table->GetNumber("ts",0.0);
  table->PutNumber("pipeline", 3);
 m_leftfollowermotor.Follow(m_leftleadmotor);
 m_rightfollowermotor.Follow(m_rightleadmotor);
}


void Robot::TeleopPeriodic() {
Ahorz = table->GetNumber("tx",0.0);
//double encoder = TurretTest->GetSelectedSensorPosition(0);
//double turn = (encoder/426) * 360; 


/*f (turn + Ahorz > 90 || turn + Ahorz < -90){
  TurretTest->Set(ControlMode::PercentOutput, 0);
}
else if (m_stick->GetRawButton(5) == 1){
    double rot = (((Ahorz / 180)*14.125) * M_PI);
    double r1 = rot/3;
    double t1 = r1/(2 * M_PI);
    double rf1 = (t1 * 8.68);

    double rotation = (Ahorz/360) * 426;

    SmartDashboard::PutNumber("horizontal", Ahorz);
    SmartDashboard::PutNumber("nums", rotation);

     errorv = rotation;
        interv = interv + errorv;  

        if(errorv == 0)
            {
                interv = 0;
            }
        else if ((interv > kP)||(interv < -kP))
            {
                interv = 0;
            }
        derav = errorv - preverrorv;
        preverrorv =  errorv;
        speedv = (kP * errorv) + (kI * interv) + (kD * derav);
        
        double wspeed = speedv;
        TurretTest->Set(ControlMode::PercentOutput, wspeed);

}
else{
  TurretTest->Set(ControlMode::PercentOutput, m_stick->GetZ()/2);
}*/
double top = SmartDashboard::GetNumber("top", -1);
double bottom = SmartDashboard::GetNumber("bottom", 1);
/*if(m_stick->GetRawButtonPressed(1) == 1){
  state = 1;
}
else if(m_stick->GetRawButtonPressed(2) == 1){
  state = 2;
}
else if(m_stick->GetRawButtonPressed(3) == 1){
  state = 3;
}
else if(m_stick->GetRawButtonPressed(4) == 1){
  state = 4;}
if(state == 1){
  Topfly->Set(ControlMode::PercentOutput, 0.2);
  Botfly->Set(ControlMode::PercentOutput, -0.2);
}
else if(state == 2){
  Topfly->Set(ControlMode::PercentOutput,0.3);
  Botfly->Set(ControlMode::PercentOutput, -0.3);
}
else if(state == 3){
  Topfly->Set(ControlMode::PercentOutput,0.5);
  Botfly->Set(ControlMode::PercentOutput,-0.5);
  state = 3;
}
else if(state == 4){
  Topfly->Set(ControlMode::PercentOutput,top);
  Botfly->Set(ControlMode::PercentOutput,-bottom);
}
else{
   Topfly->Set(ControlMode::PercentOutput,0);
  Botfly->Set(ControlMode::PercentOutput, 0);
}*/
/*if(-((m_stick->GetRawAxis(4) - 1)/2 < 0)){
  Indexer->Set(ControlMode::PercentOutput,((m_stick->GetRawAxis(4)+1)/2));
}
else{
  Indexer->Set(ControlMode::PercentOutput,0);
}*/
//if(-(m_stick->GetRawAxis(3) - 1)/2 < 0)){
   //xIntake1->Set(ControlMode::PercentOutput,-((m_stick->GetRawAxis(3)+1)/2));
   //Intake1->Set(ControlMode::PercentOutput, -((m_stick->GetRawAxis(3)+1) / 2));
  //Intake2->Set(ControlMode::PercentOutput,((m_stick->GetRawAxis(3)+1)/2));

//}
//else{
  //Intake1->Set(ControlMode::PercentOutput,0);
  //Intake2->Set(ControlMode::PercentOutput,0);
//}
//if((m_stick->GetRawAxis(3)-1)/2 < 0){

//}
//TurretTest->Set(ControlMode::PercentOutput, m_stick->GetZ()/2);

m_robotDrive.ArcadeDrive(-m_stick2->GetY(), m_stick2->GetZ());

}


void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
