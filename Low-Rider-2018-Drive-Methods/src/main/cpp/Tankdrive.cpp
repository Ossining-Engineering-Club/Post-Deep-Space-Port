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
Usonic(UsonicPort)

{
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
void Tankdrive::DirectDrivePID(float right, float left, float minLoopTimeMs){
	
}
bool lastEnable = false;
double LastTime = 0.0;
double Time = 0.0;
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
void Tankdrive::SetThrottle(float Ithrottle)
{
	throttle = (1 - Ithrottle) / 2;
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
void Tankdrive::DirectDrivePID(float right, float left, float minLoopTimeMs){
	
}
bool lastEnable = false;
double LastTime = 0.0;
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
		if (limelight.IsTargetFound()))
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
void Tankdrive::SetThrottle(float Ithrottle)
{
	throttle = (1 - Ithrottle) / 2;
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

	if (Usonic.GetRange() < 15)
		USGood = 0;

	while(((((fabs(LWEncoder.GetPosition()) + fabs(RWEncoder.GetPosition())) / 2) < Maxdistance)
			&& (Usonic.GetRange() > USrange  || !USGood)) && AutoTimer.Get() <= TimeOut)
	{
		vision.Update();
		if (limelight.IsTargetFound())
		{
			Sample = limelight.GetXOffset();
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

