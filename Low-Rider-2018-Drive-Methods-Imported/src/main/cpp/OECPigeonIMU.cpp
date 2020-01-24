#include "OECPigeonIMU.h"

OECPigeonIMU::OECPigeonIMU(int canPort){
    pigeonGyro = new ctre::phoenix::sensors::PigeonIMU(canPort);
}
OECPigeonIMU::OECPigeonIMU(ctre::phoenix::motorcontrol::can::TalonSRX *talonSRX){
    pigeonGyro = new ctre::phoenix::sensors::PigeonIMU(talonSRX);
}

void OECPigeonIMU::ResetAngle(){
    pigeonGyro->SetYaw(0.0);
}

void OECPigeonIMU::BootCalibrate(){
    pigeonGyro->EnterCalibrationMode(ctre::phoenix::sensors::PigeonIMU::CalibrationMode::BootTareGyroAccel, 6000);
}

double OECPigeonIMU::GetYaw(){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
        return ypr[0];
}

double OECPigeonIMU::GetPitch(){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
        return ypr[1];
}

double OECPigeonIMU::GetRoll(){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
        return ypr[2];
}