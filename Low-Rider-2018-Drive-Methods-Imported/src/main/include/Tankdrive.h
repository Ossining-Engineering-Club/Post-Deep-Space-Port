#pragma once
#include <frc/Timer.h>
#include <frc/DigitalInput.h>
#include <frc/Jaguar.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include "Ultrasonic.h"
#include "OECPigeonIMU.h"
#include "Vision.h"
#include "Limelight.h"
#include "OECPIDController.h"
#include "PathReader.h"

using namespace frc;
class Tankdrive
{
public: // for functions
	Tankdrive(unsigned int UsonicPort);
	void Drive(float left, float right);
	void DirectDrive(float left, float right);

	void DriveR(double power);
	void DriveL(double power);

	int DirectDrivePID(float leftRPM, float rightRPM, bool reset); // 0 - PID ran on neither, 1 - PID ran on right only, 2 - PID ran on left only, 3 - PID ran on both
	void DrivePositionPID(float leftPos, float rightPos, float lRPM, float rRPM, bool reset);

	void DrivePath(std::string leftFile, std::string rightFile);

	int TeleAimLimelight(float speed, bool enable);
	int TeleDriveLimelight(float USrange, float speed, float bias, bool enable);

	void SetThrottle(float Ithrottle);

	void AutoDriveGyro(float distance, float speed, float TimeOut);
	void AutoDriveGyro(float distance, float speed, float TimeOut, bool startup);
	void AutoDriveGyroLimit(float distance, float speed, float TimeOut, DigitalInput& LimitLift, Jaguar& Lift);
	void AutoTurnGyroBoth(float angle, float speed, float TimeOut);
	void AutoTurnGyro(float angle, float speed, float TimeOut);
	void AutoDriveGyroUS(float, float, float);

	int AutoDriveLimelight(float USrange, float speed, float Maxdistance, float TimeOut);

	bool IsLimit();

	double GetREncoder();
	double GetLEncoder();
	void ResetEncoders();
	void ResetGyro();

	double GetAngle();
	void GetUSSample();
	double GetUSRange();
private: 
	rev::CANSparkMax LeftF;
	rev::CANSparkMax RightF;
	rev::CANSparkMax LeftB;
	rev::CANSparkMax RightB;


	rev::CANEncoder LWEncoder;
	rev::CANEncoder RWEncoder;

	OECPigeonIMU Gyro;

	Timer AutoTimer;
	Limelight limelight;
	USSensor Usonic;
	
	float VisionX;
	float throttle;

	OECPIDController rdbSpeedController;
	OECPIDController ldbSpeedController;
	Timer RPMTimer;

	OECPIDController ldbPosController;
	OECPIDController rdbPosController;

	PathReader pathReader;

	//Vision:
	int returnC;
	float Sample, LastSample;
	float Integral;
	float Derivative;
	float Turn;
	bool USGood;

//Variables for RPM PIDs:
//Last reocrded position of left and right motors
double rLastPosition;
double lLastPosition;

//Time at the last recorded position of left and right motors
double rTimeLastChange;
double lTimeLastChange;

//Left and right PID correction
double rCorrection = 0.0;
double lCorrection = 0.0;

bool speedIsFirstRun;

double lsP = 0.0;
double lsI = 0.0;
double lsD = 0.0;
double lsLastError = 0.0;

double rsP = 0.0;
double rsI = 0.0;
double rsD = 0.0;
double rsLastError = 0.0;

double avgLRPM = 0.0;
double avgRRPM = 0.0;

};
