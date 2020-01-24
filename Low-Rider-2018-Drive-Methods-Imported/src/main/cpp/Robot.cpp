#include "Robot.h"

Robot::Robot():
    tankdrive(0),
    limelight(),
    arm(dash),
    intake(),
    lift(dash),
    stilts(),
    stickLeft(0),
    stickRight(1),
    stickUtil(2),
    ledRing(2),
    lidar(9)
{
    dash->init();
}
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
}

void Robot::AutonomousPeriodic() {
    tankdrive.DirectDrivePID(1000, 200, false); //for tuning speed PIDs
}

void Robot::TeleopInit() {
}


int armMode = 0;
double liftThrottle = 0.0;
int visionEnd = 4;


void Robot::TeleopPeriodic() {

    dash->PutNumber("Lidar Distance", lidar.GetDistance());
    
    //Throttles:
    liftThrottle = stickUtil.GetZ() / -2.0 + 0.5;
    tankdrive.SetThrottle(stickLeft.GetZ());
    

    //Select Vision or normal driving
    if(stickRight.GetTrigger()){
        ledRing.Set(LED_VALUE);
    }
    else{
        ledRing.Set(0.0);
    }
    visionEnd = tankdrive.TeleDriveVision(8.0, 0.3*stickRight.GetY(), stickRight.GetX(), stickRight.GetTrigger());
    if(!stickRight.GetTrigger()){
        tankdrive.Drive(stickLeft.GetY(), stickRight.GetY());
    }

    //Arm and lift mode selection
    if(abs(stickUtil.GetY()) > ARM_MANUAL_THRES || stickUtil.GetButton(2) || stickUtil.GetButton(3))
       { armMode = 0;
       dash->PutString("arm mode", "Manual");
       }
    else if(stickUtil.GetButton(1))
    {
        armMode = 1;
        dash->PutString("arm mode", "Load");
}
    else if(stickUtil.GetButton(7))
        armMode = 2;
    else if(stickUtil.GetButton(6))
        armMode = 3;
    else if(stickUtil.GetButton(8))
        armMode = 4;
    /*else if(visionEnd == 2)
        armMode = 5;*/

    if(armMode == 0){
        
        arm.SetPower(stickUtil.GetY(), stickUtil.GetButton(10));

        if(stickUtil.GetButton(3)){
            lift.SetPower(liftThrottle);
        }
        else if(stickUtil.GetButton(2)){
            lift.SetPower(-1.0 * liftThrottle);
        }
        else{
            lift.SetPower(0.0);
        }
    }

    else if(armMode == 1){
        dash->PutString("arm state", "Load");
        arm.SetToPosition(0.4, ARM_PICKUP_POS);
        lift.SetToPosition(0.3, LIFT_PICKUP_POS);
    }
    else if(armMode == 2){
        arm.SetToPosition(0.4, ARM_HOLD_POS);
        lift.SetToPosition(0.3, LIFT_HOLD_POS);
    }
    else if(armMode == 3){
        arm.SetToPosition(0.4, ARM_LEV_2_POS);
        lift.SetToPosition(0.55, LIFT_LEV_2_POS);
    }
    else if(armMode == 4){
        arm.SetToPosition(0.4, ARM_LEV_3_POS);
        lift.SetToPosition(0.55, LIFT_LEV_3_POS);
    }
    else if(armMode == 5){
        //arm.SetToPosition(0.4, ARM_CARGO1_POS);
        //lift.SetToPosition(0.3, LIFT_CARGO1_POS);
    }

}


void Robot::TestInit() {

}
void Robot::TestPeriodic() {
    
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
