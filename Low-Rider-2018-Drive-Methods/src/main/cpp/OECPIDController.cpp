#include "OECPIDController.h"

double isFirstRun = true;

OECPIDController::OECPIDController():
pidTimer()
{
    isFirstRun = true;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    correction = 0.0;
    lastError = 0.0;
    lastTime = 0.0;
}

void OECPIDController::SetConstants(double coefP, double coefI, double coefD, double MaximumCorrection)
{
    kP = coefP;
    kI = coefI;
    kD = coefD;
    maxCorrection = fabs(MaximumCorrection);
}
void OECPIDController::SetIntegral(double IVal){
    I = IVal;
}
double OECPIDController::GetCorrection(double error)
{
    double time = GetTimeMillis();
    double IIncrement;
    // Special case for first run: performs necessary setup steps and returns 0 correction:
    if(isFirstRun)
    {
        pidTimer.Start();
        lastTime = GetTimeMillis();
        lastError = error;
        isFirstRun = false;
        return 0.0;
    }

    P = kP * error;
    D = kD * ((error - lastError) / (time - lastTime));
    IIncrement = ((error + lastError) / 2) * (time - lastTime);

    //Don't add to the integral if correction is already at the max
    if(fabs(kP * P + kI * I + kD * D + kI * IIncrement) >= maxCorrection){
        correction = maxCorrection * fabs(kP * P + kI * I + kD * D + kI * IIncrement)/(kP * P + kI * I + kD * D + kI * IIncrement);
    }

    else{
        I += IIncrement;
        correction = kP * P + kI * I + kD * D;
    }

    lastTime = time;
    lastError = error;

    return correction;
}

double OECPIDController::GetTimeMillis()
{
    return pidTimer.Get()*1000.0;
}