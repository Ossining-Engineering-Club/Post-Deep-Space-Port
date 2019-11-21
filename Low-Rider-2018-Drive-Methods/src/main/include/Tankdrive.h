#pragma once
#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include "Ultrasonic.h"
#include "OECPigeonIMU.h"
#include "Vision.h"

using namespace frc;
class Tankdrive
{
public: // for functions
	Tankdrive(unsigned int UsonicPort);
	void Drive(float left, float right);
	void DirectDrive(float left, float right);
	void DirectDrivePID(float left, float right, float minLoopTimeMs, bool reset);
	int TeleDriveVision(float USrange, float speed, float bias, bool enable);
	void SetThrottle(float Ithrottle);

	void AutoDriveGyro(float distance, float speed, float TimeOut);
	void AutoDriveGyro(float distance, float speed, float TimeOut, bool startup);

	void AutoDriveGyroLimit(float distance, float speed, float TimeOut, DigitalInput& LimitLift, Jaguar& Lift);
	void AutoTurnGyroBoth(float angle, float speed, float TimeOut);
	void AutoTurnGyro(float angle, float speed, float TimeOut);
	void AutoDriveGyroUS(float, float, float);
	int AutoDriveVision(float USrange, float speed, float Maxdistance, float TimeOut);

	bool IsLimit();

	double GetREncoder();
	double GetLEncoder();
	void ResetEncoders();
	void ResetGyro();

	double GetAngle();
	void GetUSSample();
	double GetUSRange();
private: // for variables
//	Spark Left;	// change back fro worlds
//	Spark Right;
	rev::CANSparkMax LeftF;
	rev::CANSparkMax RightF;
	rev::CANSparkMax LeftB;
	rev::CANSparkMax RightB;

//	VictorSP Left;
//	VictorSP Right;
//	Encoder LWEncoder;
//	Encoder RWEncoder;
	rev::CANEncoder LWEncoder;
	rev::CANEncoder RWEncoder;
//	AnalogGyro Gyro;
	OECPigeonIMU Gyro;
	Timer AutoTimer;
	Vision vision;
	USSensor Usonic;
	float VisionX;
	float throttle;

	//Vision:
	int returnC;
	float Sample, LastSample;
	float Integral;
	float Derivative;
	float Turn;
	bool USGood;

};
