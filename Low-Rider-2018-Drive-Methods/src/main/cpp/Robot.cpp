#include "Robot.h"

Robot::Robot():
    tankdrive(0),
    stickLeft(0),
    stickRight(1),
    ledRing(2)
{}
void Robot::RobotInit()
{
   msLifeCam1 = CameraServer::GetInstance()->StartAutomaticCapture(0);
   msLifeCam1.SetResolution(XRESOLUTION, YRESOLUTION);
   msLifeCam1.SetBrightness(BRIGHTNESS);
   msLifeCam1.SetExposureManual(EXPOSURE);
   msLifeCam1.SetWhiteBalanceManual(WBM);
   msLifeCam1.SetFPS(FPS);
}

void Robot::AutonomousInit() {
    ledRing.Set(LED_VALUE);
}

void Robot::AutonomousPeriodic() {
    if(stickRight.GetTrigger()){
    tankdrive.AutoDriveVision(12.0, 0.2, 96.0, 15.0);
    }
    tankdrive.SetThrottle(stickLeft.GetZ());
    tankdrive.Drive(stickLeft.GetY(), stickRight.GetY());

}

void Robot::TeleopInit() {
    ledRing.Set(0.0);
}

void Robot::TeleopPeriodic() {
    tankdrive.SetThrottle(stickLeft.GetZ());
    tankdrive.Drive(stickLeft.GetY(), stickRight.GetY());
}


void Robot::TestInit() {

}
void Robot::TestPeriodic() {
    
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
