
#pragma once
#include <memory>
#include <algorithm>
#include <vector>
#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class Limelight
{
public:
	Limelight();

	void Update();						// Update values

	double IsTargetFound(); 				// returns 0.0 if not found 1.0 if found

	double GetArea();	// returns the area
	double GetXOffset();		// returns the x value of the tracked image
	double GetYOffset();		// returns the y value of the tracked image
	double GetHeight();	// returns the height
	double GetWidth();	// returns the width
	void SetVisionModeOn(bool VisionModeOn);


private:
	std::shared_ptr<NetworkTable> Table;
	double tv;
	double area;
	double X;
	double Y;
	double Height;
	double Width;
	unsigned int ResX;
	unsigned int ResY;
	unsigned int FullImageArea;
	bool found; 	// this is for the filter method
};
