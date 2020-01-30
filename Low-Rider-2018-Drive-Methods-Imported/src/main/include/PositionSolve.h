#pragma once

#include "Limelight.h"
#include "OECPigeonIMU.h"
#include "LidarLite.h"
#include "Constants.h"
#include <vector>

using namespace frc;

class PositionSolve{
    private:
        double LimelightAngle;
        double LimelightDistance;
        double GyroAngle;
        double LidarDistance;
        LidarLite lidar{9};
        Limelight limelight{};
        OECPigeonIMU pigeonIMU{30};

    public:
        PositionSolve();
        double GetTargetAngle();
        double GetDistLimelight();
        double GetVTHeight();
        double GetDistLidar();
        double GetDistDiff();
        std::vector<double> GetTargetRelativePosition();
};

