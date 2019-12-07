#include "Tankdrive.h"
#include <frc/WPILib.h>
// Convencion: Teleob gets joystick vals, AUTO: feed positive vals
Tankdrive::Tankdrive(unsigned int UsonicPort):

LeftF(25, rev::CANSparkMax::MotorType::kBrushless),
RightF(27, rev::CANSparkMax::MotorType::kBrushless),
LeftB(26, rev::CANSparkMax::MotorType::kBrushless),
RightB(28, rev::CANSparkMax::MotorType::kBrushless),
LWEncoder(LeftF, rev::SensorType::kHallSensor, 1),	
RWEncoder(RightF, rev::SensorType::kHallSensor, 1),
Gyro(30),
AutoTimer(),
vision(XRESOLUTION, YRESOLUTION),
limelight(),
Usonic(UsonicPort),
ldbSpeedController(),
rdbSpeedController(),
ldbPosController(),
rdbPosController(),
RPMTimer()

{
	RPMTimer.Start();
	ldbSpeedController.SetConstants(DBS_P, DBS_I, DBS_D, DBS_MAX);
	rdbSpeedController.SetConstants(DBS_P, DBS_I, DBS_D, DBS_MAX);
	ldbPosController.SetConstants(DBP_P, DBP_I, DBP_D, DBP_MAX);
	rdbPosController.SetConstants(DBP_P, DBP_I, DBP_D, DBP_MAX);
	Gyro.ResetAngle();
	LeftF.GetEncoder();
	LeftB.GetEncoder();
	RightF.GetEncoder();	RightB.GetEncoder();
	throttle = 0.0;
	VisionX = 0.0;
	LWEncoder.SetPositionConversionFactor(-1.0 * ENCODERCONST);
	RWEncoder.SetPositionConversionFactor(ENCODERCONST);
}

void Tankdrive::Drive(float left, float right)
{
	// Limit left and right inputs to between -1 and 1
	if(left > 1.0)
		left = 1.0;
	else if(left < -1.0)
		left = -1.0;
	if(right > 1)
		right = 1.0;
	else if(right < -1.0)
		right = -1.0;
	LeftF.Set(left * throttle * -1.0);
	LeftB.Set(left * throttle * -1.0);		// becuase joystick values of inversed!!!!
	RightF.Set(right * throttle);
	RightB.Set(right * throttle);
}
void Tankdrive::DirectDrive(float left, float right)
{
	if(left > 1.0)
		left = 1.0;
	else if(left < -1.0)
		left = -1.0;
	if(right > 1.0)
		right = 1.0;
	else if(right < -1.0)
		right = -1.0;
	LeftF.Set(left);
	LeftB.Set(left);		// becuase joystick values of inversed!!!!
	RightF.Set(right * -1.0);
	RightB.Set(right * -1.0);
}

void Tankdrive::DriveR(double power){
	if(power > 1.0){
		power = 1.0;
	}
	if(power < 1.0){
		power = 1.0;
	}
	RightF.Set(power * -1.0);
	RightB.Set(power * 1.0);
}
void Tankdrive::DriveL(double power){
	if(power > 1.0){
		power = 1.0;
	}
	if(power < 1.0){
		power = 1.0;
	}
	LeftF.Set(power);
	LeftB.Set(power);
}

//Variables for RPM PIDs
double rLastPosition;
double lLastPosition;
double rTimeLastChange;
double lTimeLastChange;
double lastTime;

int Tankdrive::DirectDrivePID(float left, float right, bool reset){
	bool ranl = false; //Variable to tell if the left side PID ran
	bool ranr = false; //Variable to tell if the right side PID ran
	double currTime = RPMTimer.Get();
	double rRPM = 0.0;
	double lRPM = 0.0;
	double rPosition = Tankdrive::GetREncoder()/ENCODERCONST; //Conversion from inches to rotations
	double lPosition = Tankdrive::GetLEncoder()/ENCODERCONST; //Conversion from inches to rotations
	double errorR = 0.0;
	double errorL = 0.0;

	//Resets controllers, encoders, and variables between uses
	if(reset){
		ldbSpeedController.ResetController();
		rdbSpeedController.ResetController();
		lastTime = currTime;
		rLastPosition = rPosition;
		lLastPosition = lPosition;
		Tankdrive::ResetEncoders();
		DirectDrive(left, right);
	}

	//If the right encoder position changed from last time calculate RPM and set the right PID to run
	if(rLastPosition != rPosition){
		rRPM = (rPosition - rLastPosition)/(currTime - rTimeLastChange);
		ranr = true;
	}
	//If speed is less than ~40 rpm (1.6 in/s) assume zero speed and run the PID to keep the update rate fast
	else if(currTime - rTimeLastChange >= 0.036){
		rRPM = 0.0;
		ranr = true;
	}
	else{
		ranr = false;
	}

	//Same code as above for the left side
	if(lLastPosition != lPosition){
		lRPM = (lPosition - lLastPosition)/(currTime - lTimeLastChange);
		ranl = true;
	}
	else if(currTime - lTimeLastChange >= 0.036){ 
		lRPM = 0.0;
		ranl = true;
	}
	else{
		ranl = false;
	}

	//Calculate left and right speed error
	errorR = rRPM - right;
	errorL = lRPM - left;

	//Run the PID if a new RPM has been calculated for each side
	if(ranr &&!reset)
		Tankdrive::DriveR(rdbSpeedController.GetCorrection(errorR) + right / DB_FREE_SPEED);
	if(ranl && !reset)
		Tankdrive::DriveL(rdbSpeedController.GetCorrection(errorL) +left / DB_FREE_SPEED);
	
	return (int)ranr + (int)ranl * 2; //return a value 0-3 for which PIDs ran. 0 = none 1 = right only 2 = left only 3 = both
}


//Variables for position PIDs
int rightCount = 1;
int leftCount = 1;
void Tankdrive::DrivePositionPID(float leftPos, float rightPos, float lRPM, float rRPM, bool reset){
	
	//reset variables and controllers between uses
	if(reset){
		rightCount = 1;
		leftCount = 1;
		ldbPosController.ResetController();
		rdbPosController.ResetController();
	}

	double currLeft = Tankdrive::GetLEncoder();
	double currRight = Tankdrive::GetREncoder();

	double errorL = currLeft - leftPos;
	double errorR = currRight - rightPos;

	//Update RPM setpoints for every 10 runs of RPM PIDs.
	if(leftCount == 10)
		lRPM += ldbPosController.GetCorrection(errorL);
	if(rightCount == 10)
		rRPM += rdbPosController.GetCorrection(errorR);

	int loopStatus = DirectDrivePID(lRPM, rRPM, reset);

	//Increment each side's count variable to count RPM PID runs based on RPM PID return value
	if(loopStatus == 1)
		rightCount++;
	if(loopStatus == 2)
		leftCount++;
	if(loopStatus == 3){
		rightCount++;
		leftCount++;

	if(rightCount > 10)
		rightCount == 1;
	if(leftCount > 10)
		leftCount == 1;
	}
}

void Tankdrive::DrivePath(std::string leftFile, std::string rightFile){
	//Read the left and right path files into vectors
	pathReader.ReadFile(leftFile);
	vector<double> lPositions = pathReader.GetPositions();
	vector<double> lVelocities = pathReader.GetVelocities();

	pathReader.ReadFile(rightFile);
	vector<double> rPositions = pathReader.GetPositions();
	vector<double> rVelocities = pathReader.GetVelocities();

	Timer pathTimer;
	pathTimer.Reset();
	pathTimer.Start();

	//Resets all PIDs and encoders
	DrivePositionPID(0.0, 0.0, 0.0, 0.0, true);

	while(pathTimer.Get() < PATH_DT * lPositions.size()){
		double time = pathTimer.Get();

		//The lower and upper indices to read
		double floatIndex = time/PATH_DT;
		double floorIndex = floor(PATH_DT);
		double ceilIndex = ceil(PATH_DT);

		//Linear interpolation between the lower and upper indices
		double lVelocity = lVelocities[floorIndex] + (floatIndex-floorIndex) * (lVelocities[ceilIndex]-lVelocities[floorIndex]);
		double rVelocity = rVelocities[floorIndex] + (floatIndex-floorIndex) * (rVelocities[ceilIndex]-rVelocities[floorIndex]);
		double lPosition = lPositions[floorIndex] + (floatIndex-floorIndex) * (lPositions[ceilIndex]-lPositions[floorIndex]);
		double rPosition = rPositions[floorIndex] + (floatIndex-floorIndex) * (rPositions[ceilIndex]-rPositions[floorIndex]);

		//Conversion from in/s to rpm
		lVelocity *= 60/ENCODERCONST;
		rVelocity *= 60/ENCODERCONST;

		//Drive the wheels through the PIDs
		DrivePositionPID(lPosition, rPosition, lVelocity, rVelocity, false);
	}
}

bool lastEnable = false;
double LastTime = 0.0;
double Time = 0.0;
void Tankdrive::SetThrottle(float Ithrottle)
{
	throttle = (1 - Ithrottle) / 2;
}
int Tankdrive::TeleDriveVision(float USrange, float speed, float bias, bool enable){

	speed *= -1.0;
	
	if(enable && !lastEnable){
		returnC = 0;
		Integral = 0.0;
		LastSample = 0.0;
		LastTime = 0.0;
		for (int i = 0; i < 10; i++)
			Usonic.GetSample();
		USGood = 1;

		if(speed > 1)
			speed = 1;
		else if(speed < -1)
			speed = -1;

		if(bias > 1.0)
			bias = 1.0;
		else if(bias < -1.0)
			bias = 1.0;

		AutoTimer.Reset();
		AutoTimer.Start();
		LWEncoder.SetPosition(0.0);
		RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders

		if (Usonic.GetRange() < 15)
			USGood = 0;
	}
	if(enable && (Usonic.GetRange() > USrange  || !USGood)){
		vision.Update();
		Time = AutoTimer.Get();
		if (vision.GetNumContours() != 0)
		{
			if (vision.GetNumContours() == 1) VisionX = vision.GetX(0);
			else VisionX = (vision.GetX(0) + vision.GetX(1)) /2;
			Sample = VisionX - ((XRESOLUTION/2) * (1-bias/2));		//map a bias of 1 to the left quarter and -1 to the right quarter of the image
			Integral = Integral + ((Time-LastTime)/2)*(Sample+LastSample);
		    Derivative = (Sample - LastSample)/(Time-LastTime);
		    // If Sample, Integral and Derivative are 0, then we want go with speed on each side
		    // If Sample, Integral or Derivative are large positive, left drive = -1, right drive = 1
		    // If Sample, Integral or Derivative are large negative, left drive = 1, right drive = -1
		    // We would like the average of the two sides to be speed

		    Turn = PCONSTANT * Sample + ICONSTANT * Integral + DCONSTANT * Derivative;
			Tankdrive::DirectDrive(speed * (1 - Turn), speed * (1 + Turn));
			LastSample = Sample;
		}
		else
			Tankdrive::DirectDrive(speed,speed); //Needed to prevent crash
		Usonic.GetSample();
	}
	if ((Usonic.GetRange() <= USrange) && enable){
		returnC = 2;
		Tankdrive::DirectDrive(0.0,0.0);
	}
	if(!enable){
		returnC = 4;
	}
	LastTime = Time;
	return returnC;
}

int Tankdrive::TeleDriveLimelight(float USrange, float speed, float bias, bool enable){

	speed *= -1.0;
	
	if(enable && !lastEnable){
		returnC = 0;
		Integral = 0.0;
		LastSample = 0.0;
		LastTime = 0.0;
		for (int i = 0; i < 10; i++)
			Usonic.GetSample();
		USGood = 1;

		if(speed > 1)
			speed = 1;
		else if(speed < -1)
			speed = -1;

		if(bias > 1.0)
			bias = 1.0;
		else if(bias < -1.0)
			bias = 1.0;

		AutoTimer.Reset();
		AutoTimer.Start();
		LWEncoder.SetPosition(0.0);
		RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders

		if (Usonic.GetRange() < 15)
			USGood = 0;
	}
	if(enable && (Usonic.GetRange() > USrange  || !USGood)){
		vision.Update();
		double Time = AutoTimer.Get();
		if (limelight.IsTargetFound())
		{
			Sample = limelight.GetXOffset();					//map a bias of 1 to the left quarter and -1 to the right quarter of the image
			Integral = Integral + ((Time-LastTime)/2)*(Sample+LastSample);
		    Derivative = (Sample - LastSample)/(Time-LastTime);
		    // If Sample, Integral and Derivative are 0, then we want go with speed on each side
		    // If Sample, Integral or Derivative are large positive, left drive = -1, right drive = 1
		    // If Sample, Integral or Derivative are large negative, left drive = 1, right drive = -1
		    // We would like the average of the two sides to be speed

		    Turn = PCONSTANT * Sample + ICONSTANT * Integral + DCONSTANT * Derivative;
			Tankdrive::DirectDrive(speed * (1 - Turn), speed * (1 + Turn));
			LastSample = Sample;
		}
		else
			Tankdrive::DirectDrive(speed,speed); //Needed to prevent crash
		Usonic.GetSample();
	}
	if ((Usonic.GetRange() <= USrange ))
		returnC = 2;
	if(!enable){
		returnC = 4;
	}
}

void Tankdrive::AutoDriveGyro(float distance, float speed, float TimeOut) //Args are distance, speed
{
	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();

	LWEncoder.SetPosition(0.0);
	RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders
	Gyro.ResetAngle();
	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length

	while((((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2) < distance) && AutoTimer.Get()<=TimeOut)
	{								// was +							was -
		Tankdrive::DirectDrive((speed-(fabs(speed))*AUTOGYROCONST*Gyro.GetYaw()), speed+(fabs(speed))*AUTOGYROCONST*Gyro.GetYaw());
		Wait(0.001);
	}
	Tankdrive::DirectDrive(0.0,0.0);
}

void Tankdrive::AutoDriveGyro(float distance, float speed, float TimeOut, bool startup)
{
	const float startfrac = 0.2;
	const int startms = 350;
	float dspeed;
	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();
	LWEncoder.SetPosition(0.0);
	RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders
	Gyro.ResetAngle();

	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length
	int i = 0;
	while((((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2) < distance) && AutoTimer.Get()<=TimeOut)
	{								// was +							was -
		if( i>=startms )dspeed=speed;
		else dspeed = speed * (startfrac + (1-startfrac)*((float)i/(float)startms));
		Tankdrive::DirectDrive((dspeed-(fabs(dspeed))*AUTOGYROCONST*Gyro.GetYaw()), dspeed+(fabs(dspeed))*AUTOGYROCONST*Gyro.GetYaw());
		i++;
		Wait(0.001);
	}
	Tankdrive::DirectDrive(0.0,0.0);
}

void Tankdrive::AutoDriveGyroLimit(float distance, float speed, float TimeOut, DigitalInput& LimitLift, Jaguar &Lift)
{
	Lift.Set(0.0);
	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;
	AutoTimer.Reset();
	AutoTimer.Start();

	LWEncoder.SetPosition(0.0);
	RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders
	Gyro.ResetAngle();
	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length

	while((((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2) < distance) && AutoTimer.Get()<=TimeOut)
	{								// was +							was -
		Wait(0.001);
	}
	Tankdrive::DirectDrive(0.0,0.0);
	Lift.Set(0.0);
}

int Tankdrive::AutoDriveVision(float USrange, float speed, float Maxdistance, float TimeOut) //Args are distance, speed
{
	returnC = 0;
	Integral = 0.0;
	LastSample = 0.0;
	for (int i = 0; i < 10; i++)
		Usonic.GetSample();
	USGood = 1;

	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();
	LWEncoder.SetPosition(0.0);
	RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders

	if (Usonic.GetRange() < 15)
		USGood = 0;

	while(((((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2) < Maxdistance)
			&& (Usonic.GetRange() > USrange  || !USGood)) && AutoTimer.Get() <= TimeOut)
	{
		vision.Update();
		if (vision.GetNumContours() != 0)
		{
			if (vision.GetNumContours() == 1) VisionX = vision.GetX(0);
			else VisionX = (vision.GetX(0) + vision.GetX(1)) /2;
			Sample = VisionX - 160;
			Integral = Integral + (TIMEPERIOD/2)*(Sample+LastSample);
		    Derivative = (Sample - LastSample)/TIMEPERIOD;
		    // If Sample, Integral and Derivative are 0, then we want go with speed on each side
		    // If Sample, Integral or Derivative are large positive, left drive = -1, right drive = 1
		    // If Sample, Integral or Derivative are large negative, left drive = 1, right drive = -1
		    // We would like the average of the two sides to be speed

		    Turn = PCONSTANT * Sample + ICONSTANT * Integral + DCONSTANT * Derivative;
			Tankdrive::DirectDrive(speed * (1 - Turn), speed * (1 + Turn));
			LastSample = Sample;
		}
		else
			Tankdrive::DirectDrive(speed,speed); //Needed to prevent crash
		Usonic.GetSample();
		Wait(TIMEPERIOD);
	}
	if (((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2) >= Maxdistance)
		returnC = 1;
	else if ((Usonic.GetRange() <= USrange ))
		returnC = 2;
	else if (AutoTimer.Get() > TimeOut)
		returnC = 3;

	Tankdrive::DirectDrive(0.0,0.0);
	return returnC;
}

int Tankdrive::AutoDriveLimelight(float USrange, float speed, float Maxdistance, float TimeOut) //Args are distance, speed
{
	returnC = 0;
	Integral = 0.0;
	LastSample = 0.0;
	for (int i = 0; i < 10; i++)
		Usonic.GetSample();
	USGood = 1;

	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();
	LWEncoder.SetPosition(0.0);
	RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders

	if (Usonic.GetRange() < 15.0)
		USGood = 0;

	while(((((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2.0) < Maxdistance)
			&& (Usonic.GetRange() > USrange  || !USGood)) && AutoTimer.Get() <= TimeOut)
	{
		limelight.Update();
		if (limelight.IsTargetFound() == 1.0)
		{
			Sample = limelight.GetXOffset()*6.0;
			Integral = Integral + (TIMEPERIOD/2)*(Sample+LastSample);
		    Derivative = (Sample - LastSample)/TIMEPERIOD;
		    Turn = PCONSTANT * Sample + ICONSTANT * Integral + DCONSTANT * Derivative;
			Tankdrive::DirectDrive(speed * (1 - Turn), speed * (1 + Turn));
			LastSample = Sample;
		}
		else
			Tankdrive::DirectDrive(speed,speed); //Needed to prevent crash
		Usonic.GetSample();
		Wait(TIMEPERIOD);
	}
	if (((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2) >= Maxdistance)
		returnC = 1;
	else if ((Usonic.GetRange() <= USrange ))
		returnC = 2;
	else if (AutoTimer.Get() > TimeOut)
		returnC = 3;

	Tankdrive::DirectDrive(0.0,0.0);
	return returnC;
}

void Tankdrive::AutoTurnGyroBoth(float angle, float speed, float TimeOut)	 //Args are angle, speed
{
//	float diff = 0.0;
	AutoTimer.Reset(); AutoTimer.Start();
	LWEncoder.SetPosition(0.0);
	RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders
	Gyro.ResetAngle();
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;				// was -	was +
	if (angle > 0.0) Tankdrive::DirectDrive(speed, -1.0 * speed);
	else Tankdrive::DirectDrive(-1.0 * speed, speed);

	while (fabs(Gyro.GetYaw()) <= fabs(angle)  && AutoTimer.Get() <= TimeOut)	//When the gyroscope gives a reading below/equal to 45
	{
/*		if((diff=(fabs(angle)-fabs(Gyro.GetYaw()))/ANGTOLERANCE <= 1.0))
		{
				if (angle > 0.0) Tankdrive::DirectDrive(speed * diff, -1.0 * speed * diff);
				else Tankdrive::DirectDrive(-1.0 * speed * diff, speed * diff);
		}*/
	    Wait(0.001);

	}
	Tankdrive::DirectDrive(0.0, 0.0);
}

void Tankdrive::AutoTurnGyro(float angle, float speed, float TimeOut)	 //Args are angle, speed
{
//	float diff = 0.0;
	AutoTimer.Reset(); AutoTimer.Start();
	LWEncoder.SetPosition(0.0);
	RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders
	Gyro.ResetAngle();
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;				// was -	was +
	if (angle * speed > 0.0)
		Tankdrive::DirectDrive(1.0*speed, 0.0 * speed);
	else if (angle * speed < 0.0)
		Tankdrive::DirectDrive(0.0 * speed, 1.0*speed);
	else
		return;

	while (fabs(Gyro.GetYaw()) <= fabs(angle)  && AutoTimer.Get() <= TimeOut)	//When the gyroscope gives a reading below/equal to 45
	{
	/*	if((diff=(fabs(angle)-fabs(Gyro.GetYaw()))/ANGTOLERANCE <= 1.0))
		{
			if (angle * speed > 0.0) Tankdrive::DirectDrive(speed * diff, 0.0);
			else Tankdrive::DirectDrive(0.0, speed * diff);
		}
			    Wait(0.001);*/
	    Wait(0.001);
	}
	Tankdrive::DirectDrive(0.0, 0.0);
}

void Tankdrive::AutoDriveGyroUS(float USrange, float speed, float Maxdistance) //Args are distance, speed
{
	for (int i = 0; i < 10; i++)
		Usonic.GetSample();
	bool USGood=1;
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;
	AutoTimer.Reset(); AutoTimer.Start();
	LWEncoder.SetPosition(0.0);
	RWEncoder.SetPosition(0.0);    //Reset Wheel Encoders
	Gyro.ResetAngle();
	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length

	if (Usonic.GetRange() < 15) USGood=0;

	while(((((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2) < Maxdistance)
			&& (Usonic.GetRange() > USrange  || !USGood)) && AutoTimer.Get()<=AUTOTIMEMAX)
	{
		Tankdrive::DirectDrive(speed-AUTOGYROCONST*Gyro.GetYaw(), speed+AUTOGYROCONST*Gyro.GetYaw());
		Usonic.GetSample();
		Wait(TIMEPERIOD);
	}
	Tankdrive::DirectDrive(0.0,0.0);
}

double Tankdrive::GetREncoder()
{
	return RWEncoder.GetPosition();
}

double Tankdrive::GetLEncoder()
{
	return LWEncoder.GetPosition();
}

void Tankdrive::ResetEncoders()
{
	RWEncoder.SetPosition(0.0);
	LWEncoder.SetPosition(0.0);
}

void Tankdrive::ResetGyro()
{
	Gyro.ResetAngle();
}

double Tankdrive::GetAngle()
{
	return Gyro.GetYaw();
}

void Tankdrive::GetUSSample()
{
	Usonic.GetSample();
}
double Tankdrive::GetUSRange()
{
	return Usonic.GetRange();
}

