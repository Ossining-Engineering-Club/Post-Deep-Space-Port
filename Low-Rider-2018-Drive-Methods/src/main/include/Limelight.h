/*
 * This is the Vision class programmed by Nick Tremaroli
 * This was designed to JUST GET VALUES FROM GRIP
 * For any questions talk to Nick Trem
 */
#pragma once
#include <memory>
#include <frc/WPILib.h>
#include <algorithm>
#include <vector>
#include "Constants.h"
#include <networktables/NetworkTableInstance.h>
#define NetTable shared_ptr<NetworkTable>	// the network table uses a shared pointer, this is a smart pointer
#define Gvector vector<double>				// a vector designed for Grip

using std::vector;
using std::shared_ptr;

using namespace frc;
class Limelight
{
public:
	Limelight();

	void Update();						// Update the camera values

	bool IsTargetFound(); 				// returns the number of contours

	double GetArea();	// returns the area
	double GetXOffset();		// returns the x value of the tracked image
	double GetYOffset();		// returns the y value of the tracked image
	double GetHeight();	// returns the height
	double GetWidth();	// returns the width


private:
	NetTable Table;
	Gvector tv;
	Gvector area;
	Gvector X;
	Gvector Y;
	Gvector Height;
	Gvector Width;
	unsigned int ResX;
	unsigned int ResY;
	unsigned int FullImageArea;
	bool found; 	// this is for the filter method
};
