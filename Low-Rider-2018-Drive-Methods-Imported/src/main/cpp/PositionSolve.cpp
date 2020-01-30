#include "PositionSolve.h"
#include <math.h>

PositionSolve::PositionSolve(){
    LimelightAngle = 0.0;
    LimelightDistance = 0.0;
    GyroAngle = 0.0;
    LidarDistance = 0.0;
}
double PositionSolve::GetTargetAngle(){
    limelight.Update();
    return limelight.GetXOffset();
}
double PositionSolve::GetDistLimelight(){
    limelight.Update();
    return LIMELIGHT_DIST_CONSTANT/limelight.GetHeight();
}
double PositionSolve::GetVTHeight(){
    return limelight.GetHeight();
}
double PositionSolve::GetDistLidar(){
    return lidar.GetDistance();
}
double PositionSolve::GetDistDiff(){
    limelight.Update();
    LidarDistance = GetDistLidar();
    LimelightDistance = GetDistLimelight();
    double avgdist = (LidarDistance+LimelightDistance)*0.5;
    return abs((LidarDistance-LimelightDistance)/avgdist);
}
std::vector<double> PositionSolve::GetTargetRelativePosition(){
    limelight.Update();
    double angle = 90-GetTargetAngle()-pigeonIMU.GetYaw();
    double x = GetDistLidar()*cos(angle*3.141592654/180.0);
    double y = GetDistLidar()*sin(angle*3.141592654/180.0);
    std::vector<double> result;
    result.push_back(x);
    result.push_back(y);
    result.push_back(angle);
    return result;
}