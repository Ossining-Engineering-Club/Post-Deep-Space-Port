#include "Robot.h"

Robot::Robot():
    tankdrive(0),
    stickLeft(0),
    stickRight(1)
{}
void Robot::RobotInit()
{
   
}

void Robot::AutonomousInit() {
    
}

void Robot::AutonomousPeriodic() {
    
}

void Robot::TeleopInit() {
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
