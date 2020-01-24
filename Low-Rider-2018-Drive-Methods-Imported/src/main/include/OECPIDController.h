#pragma once
#include <frc/WPIlib.h>

class OECPIDController{
    public:
        OECPIDController();
        void ResetController();
        void SetConstants(double coefP, double coefI, double coefD, double MaximumCorrection);
        void SetIntegral(double IVal);
        double GetCorrection(double error);
    private:
        frc::Timer pidTimer;
        double GetTimeMillis();
        double isFirstRun;
        double kP;
        double kI;
        double kD;
        double maxCorrection;

        double P;
        double I;
        double D;

        double TimeMillis;

        double correction;
        double lastError;
        double lastTime;
};