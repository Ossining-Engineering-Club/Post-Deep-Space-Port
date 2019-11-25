#include <frc/WPILib.h>
#include "Limelight.h"

Limelight::Limelight()
{
	auto inst = nt::NetworkTableInstance::GetDefault();
	Table = inst.GetTable("limelight");	
	found = false;
	this->ResX = ResX;
	this->ResY = ResY;
	FullImageArea = this->ResX * this->ResY;
}

void Limelight::Update()
{
	tv = Table ->GetNumberArray("tv", wpi::ArrayRef<double>());
	area = Table->GetNumberArray("ta", wpi::ArrayRef<double>());
	X = Table->GetNumberArray("tx", wpi::ArrayRef<double>());
	Y = Table->GetNumberArray("ty", wpi::ArrayRef<double>());
	Height = Table->GetNumberArray("tvert", wpi::ArrayRef<double>());
	Width = Table->GetNumberArray("thor", wpi::ArrayRef<double>());
}


bool Limelight::IsTargetFound()
{
	return tv[0];
}

double Limelight::GetArea()
{

	return area[0];
}

double Limelight::GetXOffset()
{
	return X[0];
}

double Limelight::GetYOffset()
{
	return Y[0];
}
double Limelight::GetHeight()
{
	return Height[0];
}

double Limelight::GetWidth()
{
	return Width[0];
}


