/***************************************************************************************
 *
 * Declare the Interface for the Ossining High School Robot
 * 
 */
#pragma once
#include <frc/AnalogInput.h>	//Include the FRC Library


/*
 * Ultrasonic Sensor Class
 */

using namespace frc;
class USSensor
{
	public:
		USSensor(unsigned int channel);	//Constructor to intialize - default is off
		float GetRawRange(void);  //Gives unaveraged range value
		void GetSample(void);	//This is used for taking a sample
		float GetRange(void);	//Find the range using the sensor
	private:
		AnalogInput uschannel;
		float averagerange;		//Variable accumulates average range
};
