#pragma once
#include <ctre/Phoenix.h>

class OECPigeonIMU{
    public:
        OECPigeonIMU(int canPort);
        OECPigeonIMU(ctre::phoenix::motorcontrol::can::TalonSRX *talonSRX);
        void ResetAngle();
        void BootCalibrate();
        double GetYaw();
        double GetPitch();
        double GetRoll();
    private:
        ctre::phoenix::sensors::PigeonIMU *pigeonGyro;
};