#pragma once
#include <frc/WPIlib.h>
#include "Constants.h"
#include "Ultrasonic.h"
#include <rev/CANSparkMax.h>
using namespace frc;

class Tankdrive
{
public: // for functions
	Tankdrive(unsigned int LeftFrontchannel, unsigned int RightFrontchannel, unsigned int LeftBackchannel, unsigned int RightBackchannel, unsigned int GyroPort, unsigned int UsonicPort);
	void Drive(float left, float right);
	void DirectDrive(float left, float right);
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
	rev::CANSparkMax LeftFront;	// change back fro worlds
    rev::CANSparkMax RightFront;
    rev::CANSparkMax LeftBack;
    rev::CANSparkMax RightBack;
//	VictorSP Left;
//	VictorSP Left;
//	VictorSP Right;
	Encoder LWEncoder;
	Encoder RWEncoder;
	AnalogGyro Gyro;
	Timer AutoTimer;
//	Vision vision;
	USSensor Usonic;
	float VisionX;
	float throttle;
};