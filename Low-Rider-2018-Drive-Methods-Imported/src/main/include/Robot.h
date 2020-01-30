/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
 
 #include <frc/smartdashboard/SmartDashboard.h>
 #include <frc/Victor.h>
 #include <cscore_oo.h>
 #include "Tankdrive.h"
 #include "Constants.h"
 #include "Arm.h"
 #include "Stilts.h"
 #include "Intake.h"
 #include "Lift.h"
 #include "OECJoystick.h"
 #include "Limelight.h"
 #include "LidarLite.h"
 #include "PositionSolve.h"

using namespace frc;

class Robot : public frc::TimedRobot {
 private:
  cs::UsbCamera msLifeCam1;
  SmartDashboard *dash;
  Tankdrive tankdrive;
  Limelight limelight;
  Arm arm;
  Intake intake;
  Lift lift;
  Stilts stilts;

  OECJoystick stickLeft;
  OECJoystick stickRight;
  OECJoystick stickUtil;
  Victor ledRing;
  LidarLite lidar;
  PositionSolve solve;

  
 public:
  Robot();
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
  
};
