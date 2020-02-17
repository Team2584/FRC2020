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
double kP = 0.15, kI = 0, kD = 0,  kMaxOutput = 0.25; 
double speedw = 0.0, winchpos = 0.0, errorw = 0.0, interw = 0.0, deraw = 0.0, preverrorw = 0.0;
double wP = 0.15, wI = 0, wD = 0,  wMax = 0.25; 
double speedd = 0.0, errord = 0.0, interd = 0.0, derad = 0.0, preverrod = 0.0;
double dP = 0.1, dI = 0.0, dD = 0.0005, dmax = 0.1;
int state = 0, top = 0, bottom = 0;
int winchStatrt = 0;
int Autostate = 0, shot = 0, Atimer = 0;


double leftleadmotorID = 3, rightleadmotorID = 1, leftfollowmotorID = 4 , rightfollowermotorID = 2, SparkBotFlyID = 9, topFly = 8, winchID = 12; 
  rev::CANSparkMax m_leftleadmotor{leftleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftfollowermotor{leftfollowmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightleadmotor{rightleadmotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightfollowermotor{rightfollowermotorID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax SparkBotFly{SparkBotFlyID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax SparkTopFly{topFly, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax winch{winchID, rev::CANSparkMax::MotorType::kBrushless};

  frc::Joystick *m_stick;
  frc::Joystick *m_stick2;

  TalonSRX *TurretTest;
  TalonSRX *Topfly;
  TalonSRX *Botfly;
  VictorSPX *Indexer;
  VictorSPX *Intake1;
  VictorSPX *Intake2;
  Servo *Turn;
  Servo *Wheel;
  TalonSRX *Color;
  VictorSPX *Lift;
  DigitalInput *TopLimit;
  DigitalInput *BottomLimit;
  int statel;
  
std::shared_ptr<NetworkTable> table;

 rev::CANEncoder m_encoder = m_rightleadmotor.GetEncoder();
 rev::CANEncoder m_encoder2 = m_leftleadmotor.GetEncoder();
 rev::CANEncoder winchE = winch.GetEncoder();

frc::DifferentialDrive m_robotDrive{m_leftleadmotor, m_rightleadmotor};
frc::DifferentialDrive m_robotDrive2{m_leftfollowermotor,m_rightfollowermotor};
double Ahorz, HorzL, VertL,Servo1, speedF, speeddr, speeddt, Epos, Lpos;
int statea, stateS,  ServoS, ServoS2, timer;

static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(272, 204);
        camera.SetFPS(22);
    }

void Robot::RobotInit() {
  //m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  //m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  //frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

 

  Ahorz = 0;
  HorzL = 0;
   double tA = 0, tS = 0;
  VertL = 0;
  statea = 0;
  stateS = 0;

  TurretTest = new TalonSRX(6);
  

  Indexer = new VictorSPX(4);

  Intake1 = new VictorSPX(1);
  Intake2 = new VictorSPX(2);

  Lift = new VictorSPX(3);
  Color = new TalonSRX(11);

  Turn = new Servo(1);
  Wheel = new Servo(0);

  m_stick = new Joystick(0);
  m_stick2 = new Joystick(1);

  TopLimit = new DigitalInput(0);
  BottomLimit = new DigitalInput(1);
  statel = 0;
  timer = 0;
  Lpos = 0;

SmartDashboard::PutNumber("horizontal", 0);
SmartDashboard::PutNumber("nums", 0);
SmartDashboard::PutNumber("top", top);
SmartDashboard::PutNumber("bottom", bottom);
SmartDashboard::PutNumber("P", dP);
SmartDashboard::PutNumber("I", dI);
SmartDashboard::PutNumber("D", dD); 
SmartDashboard::PutNumber("*Speed", dmax); 
SmartDashboard::PutNumber("VertL", VertL);
SmartDashboard::PutNumber("Servo angle", 0);
SmartDashboard::PutNumber("LimeLight Difference", 0.0);
SmartDashboard::PutNumber("Intake voltage", 0.0);
SmartDashboard::PutNumber("Toplimit", 0.0);
SmartDashboard::PutNumber("bottomlimit", 0.0);
SmartDashboard::PutNumber("servo", 0.0);
SmartDashboard::PutNumber("Am I a fool", 0.0);
}


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
 
 m_encoder.SetPosition(0.0);
  Autostate = 0;
  shot = 0;
  Atimer = 50;
  //if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  //} //else {
    // Default Auto goes here
  //}
}

void Robot::AutonomousPeriodic() {
  
if (Autostate == 0){
 if(Atimer == 0){
   Autostate = 1;
   Atimer = 100;
 }
 else{
   SparkTopFly.Set(-0.6);
   SparkBotFly.Set(0.6);
   Intake1->Set(ControlMode::PercentOutput, 0);
   Intake2->Set(ControlMode::PercentOutput, 0);
   Indexer->Set(ControlMode::PercentOutput, 0);
   Atimer -= 1;
 }
}
else if(Autostate == 1){
  if(Atimer == 0){
    shot += 1;
    if(shot == 1){
      Autostate = 2;
    }
    else{
    Autostate = 0;
    Atimer = 125;
    }
  }
  else{
    SparkTopFly.Set(-0.6);
   SparkBotFly.Set(0.6);
   Intake1->Set(ControlMode::PercentOutput, 1);
   Intake2->Set(ControlMode::PercentOutput, -1);
   Indexer->Set(ControlMode::PercentOutput, 1);
   Atimer -= 1;
  }
}
else{
   SparkTopFly.Set(-0.6);
   SparkBotFly.Set(0.6);
   Intake1->Set(ControlMode::PercentOutput, 1);
   Intake2->Set(ControlMode::PercentOutput, -1);
   Indexer->Set(ControlMode::PercentOutput, 1);

  
dP = SmartDashboard::GetNumber("P", dP);
dI = SmartDashboard::GetNumber("I", dI);
dD = SmartDashboard::GetNumber("D", dD); 
double dmax = SmartDashboard::GetNumber("*Speed", dmax); 

 double encode = m_encoder.GetPosition();
 SmartDashboard::PutNumber("nums", encode);
  
    double far = ( -5 / (0.5 * M_PI));
    double rf1 = (far * 8.68);
    

     //errord = (1.0345 * rf1) - encode;
     errord = rf1 - encode;
        interd = interd + errord;  

        if(errord == 0)
            {
                interd = 0;
            }
        else if ((interd > dI)||(interd < -dI))
            {
                interd = 0;
            }
        derad = errord - preverrod;
        preverrod =  errord;
        speedd = (dP * errord) + (dI * interd) + (dD * derad);
        
        double dspeed = speedd *dmax;
        m_rightleadmotor.Set(dspeed);
        m_rightfollowermotor.Set(dspeed);
        m_leftleadmotor.Set(-dspeed);
        m_leftfollowermotor.Set(-dspeed);
 
  }

Ahorz = table->GetNumber("tx",0.0);
VertL = table->GetNumber("tvert", 0.0);
HorzL = table->GetNumber("thor",0.0);
   

    double rotation = ((Ahorz + 1) /360) * 426;

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
        
        double wspeed = kMaxOutput * speedv;
        TurretTest->Set(ControlMode::PercentOutput, wspeed);


}

 
void Robot::TeleopInit() {
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  Ahorz = table->GetNumber("tx",0.0);
  HorzL = table->GetNumber("thor",0.0);
  double tA = table->GetNumber("ta",0.0);
  double tS = table->GetNumber("ts",0.0);
  VertL = table->GetNumber("tvert", 0.0);
  table->PutNumber("pipeline", 3);
  statea = 0;
  stateS = 0;
  speedF = 0;
  speedd = 0;
  speeddt = 0;

  ServoS = 0;

  ServoS2 = 0;

  Epos = 0;
  
  winchStatrt = 0;
  
  
  }


void Robot::TeleopPeriodic() {
/*if(m_stick2->GetRawButtonPressed(14) == 1){
  statea = 1;
}
double angle = Turn->Get();
SmartDashboard::PutNumber("Servo angle", angle);

dP = SmartDashboard::GetNumber("P", dP);
dI = SmartDashboard::GetNumber("I", dI);
dD = SmartDashboard::GetNumber("D", dD); 
double dmax = SmartDashboard::GetNumber("*Speed", dmax); 

 double encode = m_encoder.GetPosition();
 SmartDashboard::PutNumber("nums", encode);
  if((statea == 1)){
    double far = (10 / (0.5 * M_PI));
    double rf1 = (far * 8.68);
    

     errord = rf1 - encode;
        interd = interd + errord;  

        if(errord == 0)
            {
                interd = 0;
            }
        else if ((interd > dI)||(interd < -dI))
            {
                interd = 0;
            }
        derad = errord - preverrod;
        preverrod =  errord;
        speedd = (dP * errord) + (dI * interd) + (dD * derad);
        
        double dspeed = speedd;
       m_rightleadmotor.Set(dspeed * dmax);
        m_rightfollowermotor.Set(dspeed* dmax);
        m_leftleadmotor.Set(-dspeed * dmax);
        m_leftfollowermotor.Set(-dspeed * dmax); 

}
*/
/*
       d8888 d8b               d8b                   
      d88888 Y8P               Y8P                   
     d88P888                                         
    d88P 888 888 88888b.d88b.  888 88888b.   .d88b.  
   d88P  888 888 888 "888 "88b 888 888 "88b d88P"88b 
  d88P   888 888 888  888  888 888 888  888 888  888 
 d8888888888 888 888  888  888 888 888  888 Y88b 888 
d88P     888 888 888  888  888 888 888  888  "Y88888 
                                                 888 
                                            Y8b d88P 
                                             "Y88P" 
*/
/*
This is autimatic Aiming, later this will have a function that calculates what speeds the flywheel should be spinning at and what offset of the limelight should be,
but for now it is just horizontal aiming
*/
Ahorz = table->GetNumber("tx",0.0);
VertL = table->GetNumber("tvert", 0.0);
double encoder = TurretTest->GetSelectedSensorPosition(0);
double turn = (encoder/426) * 360; 

double difference = SmartDashboard::GetNumber("LimeLight Difference", 0.0);
SmartDashboard::PutNumber("VertL", VertL);
turn += difference;
if (turn + Ahorz > 160 || turn + Ahorz < -90){
  TurretTest->Set(ControlMode::PercentOutput, 0);
}
else if (m_stick2->GetRawButton(1) == 1){
   

    double rotation = ((Ahorz + difference) /360) * 426;

    SmartDashboard::PutNumber("horizontal", Ahorz);
    

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
        
        double wspeed = kMaxOutput * speedv;
        TurretTest->Set(ControlMode::PercentOutput, wspeed);

}
else{

  TurretTest->Set(ControlMode::PercentOutput, m_stick2->GetZ()/2);
}
/*
888b     d888       .d8888b.  888                        888    d8b                   
8888b   d8888      d88P  Y88b 888                        888    Y8P                   
88888b.d88888      Y88b.      888                        888                          
888Y88888P888       "Y888b.   88888b.   .d88b.   .d88b.  888888 888 88888b.   .d88b.  
888 Y888P 888          "Y88b. 888 "88b d88""88b d88""88b 888    888 888 "88b d88P"88b 
888  Y8P  888            "888 888  888 888  888 888  888 888    888 888  888 888  888 
888   "   888      Y88b  d88P 888  888 Y88..88P Y88..88P Y88b.  888 888  888 Y88b 888 
888       888       "Y8888P"  888  888  "Y88P"   "Y88P"   "Y888 888 888  888  "Y88888 
                                                                                  888 
                                                                             Y8b d88P 
                                                                              "Y88P"
*/
/*
Currently this is just to set the flywheel to different speeds for testing, later this will have a change of modes betweeen moving the speed up a down for shooting and a winch
or three preset modes for close, the line and the trench shooting
*/
double top = SmartDashboard::GetNumber("top", -1);
double bottom = SmartDashboard::GetNumber("bottom", 1);

if(m_stick2->GetRawButtonPressed(2) == 1){
  state = 1;
}
else if(m_stick2->GetRawButtonPressed(3) == 1){
  state = 3;
}
else if(m_stick2->GetRawButtonPressed(4) == 1){
  state = 4;}
else if (m_stick2->GetPOV() != -1){
  state = 0;
}
if(state == 1){
  SparkTopFly.Set(0);
  SparkBotFly.Set(0);
}
else if(state == 3){
  SparkTopFly.Set(-0.5);
  SparkBotFly.Set(0.5);
  state = 3;
}
else if(state == 4){
  SparkTopFly.Set(-top);
  SparkBotFly.Set(bottom);
}
else{
   SparkTopFly.Set(0);
  SparkBotFly.Set(0);
}
if(state == 0){
if((timer == 0)){
  if(m_stick2->GetPOV() == 0){
    if((speedF + 0.1) > 1){
      speedF = speedF;
    }
    else{
      speedF += 0.1;
      timer = 5;
    }
  }
  else if(m_stick2->GetPOV() == 180){
    if((speedF - 0.1) < 0){
      speedF = speedF;
    }
    else{
      speedF -= 0.1;
      timer = 5;
    }
  }
  else if(m_stick2->GetPOV() == 90){
    speedF = 0.0;
  }
  else if (m_stick2->GetPOV() == 270){
    speedF = 1.0;
  }
  else{
    speedF = speedF;
  }
  
  SparkTopFly.Set(-speedF);
  SparkBotFly.Set(speedF);
}
else if (timer > 0){
  timer -= 1;
}
else{
  SparkTopFly.Set(0);
  SparkBotFly.Set(0);
}
}
SmartDashboard::PutNumber("servo", m_stick2->GetPOV());
/*
8888888               888                                    
  888                 888                                    
  888                 888                                    
  888   88888b.   .d88888  .d88b.  888  888  .d88b.  888d888 
  888   888 "88b d88" 888 d8P  Y8b `Y8bd8P' d8P  Y8b 888P"   
  888   888  888 888  888 88888888   X88K   88888888 888     
  888   888  888 Y88b 888 Y8b.     .d8""8b. Y8b.     888     
8888888 888  888  "Y88888  "Y8888  888  888  "Y8888  888 
*/
/*
This runs the indexer up a down for loading and shooting balls, in later versions it will not move unless the flywheels are at acceptable speed
*/
  if((m_stick2->GetRawAxis(4)+1)/2 > 0){
    Indexer->Set(ControlMode::PercentOutput,((m_stick2->GetRawAxis(4)+1)/2));
  }
  else if((m_stick2->GetRawAxis(3)+1)/2 > 0){
  Indexer->Set(ControlMode::PercentOutput,-((m_stick2->GetRawAxis(3)+1)/2));
  }
  else{
    Indexer->Set(ControlMode::PercentOutput, 0);
  }

/*
8888888          888             888               
  888            888             888               
  888            888             888               
  888   88888b.  888888  8888b.  888  888  .d88b.  
  888   888 "88b 888        "88b 888 .88P d8P  Y8b 
  888   888  888 888    .d888888 888888K  88888888 
  888   888  888 Y88b.  888  888 888 "88b Y8b.     
8888888 888  888  "Y888 "Y888888 888  888  "Y8888  
*/
/*
This is the function for the intake
*/
double intakeVoltage = Intake1->GetMotorOutputVoltage();
SmartDashboard::PutNumber("Intake voltage", intakeVoltage);

if((m_stick->GetRawAxis(4)+1 > 0)){
   Intake1->Set(ControlMode::PercentOutput,((m_stick->GetRawAxis(4)+1)/2));
  
  Intake2->Set(ControlMode::PercentOutput,(-(m_stick->GetRawAxis(4)+1)/2));
}
else if((m_stick->GetRawAxis(3)+1) > 0){
  Intake1->Set(ControlMode::PercentOutput,(-(m_stick->GetRawAxis(3)+1)/2));
   
  Intake2->Set(ControlMode::PercentOutput,((m_stick->GetRawAxis(3)+1)/2));
}
else {
  Intake1->Set(ControlMode::PercentOutput,0);
  
  Intake2->Set(ControlMode::PercentOutput,0);
}
/*
8888888b.          d8b                   
888  "Y88b         Y8P                   
888    888                               
888    888 888d888 888 888  888  .d88b.  
888    888 888P"   888 888  888 d8P  Y8b 
888    888 888     888 Y88  88P 88888888 
888  .d88P 888     888  Y8bd8P  Y8b.     
8888888P"  888     888   Y88P    "Y8888 
*/


if(m_stick->GetPOV() != -1){
if(m_stick->GetPOV() == 0){
      speeddr = 0.1;
      speeddt = 0.1;
  }
  else if(m_stick->GetPOV() == 180){
      speeddr = -0.1;
      speeddt = -0.1;
  }

  else if(m_stick->GetPOV() == 90){
      speeddr = -0.1;
      speeddt = 0.1;
  }
  else if (m_stick->GetPOV() == 270){
      speeddr = 0.1;
      speeddt = -0.1;
  }
  else{
    speeddr = 0;
    speeddt = 0;
  }
   m_rightleadmotor.Set(-speeddr);
   m_rightfollowermotor.Set(-speeddr);
   m_leftleadmotor.Set(speeddt);
   m_leftfollowermotor.Set(speeddt);
}
else{
  m_robotDrive.ArcadeDrive(-m_stick->GetY(), m_stick->GetZ()*0.5);
  m_robotDrive2.ArcadeDrive(-m_stick->GetY(), m_stick->GetZ()*0.5);
}
/*
 .d8888b.           888                      d88P  .d8888b.                         888          888          
d88P  Y88b          888                     d88P  d88P  Y88b                        888          888          
888    888          888                    d88P   888    888                        888          888          
888         .d88b.  888  .d88b.  888d888  d88P    888         .d88b.  88888b.   .d88888  .d88b.  888  8888b.  
888        d88""88b 888 d88""88b 888P"   d88P     888  88888 d88""88b 888 "88b d88" 888 d88""88b 888     "88b 
888    888 888  888 888 888  888 888    d88P      888    888 888  888 888  888 888  888 888  888 888 .d888888 
Y88b  d88P Y88..88P 888 Y88..88P 888   d88P       Y88b  d88P Y88..88P 888  888 Y88b 888 Y88..88P 888 888  888 
 "Y8888P"   "Y88P"  888  "Y88P"  888  d88P         "Y8888P88  "Y88P"  888  888  "Y88888  "Y88P"  888 "Y888888
*/
/*
Pretty simple, this is the movement controls for the color wheel spinner and the Gondola movement controlled by the same function
*/
Color->Set(ControlMode::PercentOutput, m_stick2->GetX());
//also the gondola

/*
 .d8888b.  888 d8b               888      
d88P  Y88b 888 Y8P               888      
888    888 888                   888      
888        888 888 88888b.d88b.  88888b.  
888        888 888 888 "888 "88b 888 "88b 
888    888 888 888 888  888  888 888  888 
Y88b  d88P 888 888 888  888  888 888 d88P 
 "Y8888P"  888 888 888  888  888 88888P"  
*/
/*
This is driver function for the climb
*/
if(m_stick->GetRawButton(3) == 1){
 Lift->Set(ControlMode::PercentOutput, 1);
}
else if(m_stick->GetRawButton(4) == 1){
  Lift->Set(ControlMode::PercentOutput, -1); 
}
else{
  Lift->Set(ControlMode::PercentOutput, 0);
}

Turn->SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);

if(m_stick->GetRawButton(1) == 1){
  ServoS = 1;
}
if(m_stick->GetRawButton(2) == 1){
   ServoS = 2;
 }
if(ServoS == 1){
  Wheel->Set(1);
}
if(ServoS == 2){
  Wheel->Set(-0.1);
}
if(m_stick2->GetRawButton(10) == 1){
   Turn->SetSpeed(0.5);
   Lpos = Turn->GetAngle();
  
   if(Lpos >= 1){
  if(TopLimit->Get() == 1){
    Lift->Set(ControlMode::PercentOutput, -1);
  }
  else{
    Lift->Set(ControlMode::PercentOutput, 0); 
  }
   }
}
else if(m_stick2->GetRawButton(9) == 1){
  if(BottomLimit->Get() == 1){
    Lift->Set(ControlMode::PercentOutput, 1); 
  }
  else{
  Lift->Set(ControlMode::PercentOutput, 0);
  Turn->SetSpeed(0);
  }
}
else{
  Lift->Set(ControlMode::PercentOutput, 0); 
}
SmartDashboard::PutNumber("Toplimit", TopLimit->Get());
SmartDashboard::PutNumber("bottomlimit",BottomLimit->Get());
/*
888       888 d8b                   888      
888   o   888 Y8P                   888      
888  d8b  888                       888      
888 d888b 888 888 88888b.   .d8888b 88888b.  
888d88888b888 888 888 "88b d88P"    888 "88b 
88888P Y88888 888 888  888 888      888  888 
8888P   Y8888 888 888  888 Y88b.    888  888 
888P     Y888 888 888  888  "Y8888P 888  888 
*/
/* Code for the winch, making the winch idle at brake will hopefully not allow it to back drive*/
winch.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
double posi = winchE.GetPosition();
if(m_stick->GetRawButton(14) == 1){
  winch.Set(1);
}
else if(m_stick->GetRawButton(13) == 1){
 winch.Set(-0.2);
 
}
else{
    winch.Set(0);
}
}



void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

