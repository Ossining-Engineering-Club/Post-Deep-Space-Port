#include "Limelight.h"

Limelight::Limelight()
{
	Table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

void Limelight::Update()
{
	tv = Table->GetNumber("tv", 0.0);
	area = Table->GetNumber("ta", 0.0);
	X = Table->GetNumber("tx", 0.0);
	Y = Table->GetNumber("ty", 0.0);
	Height = Table->GetNumber("tvert", 0.0);
	Width = Table->GetNumber("thor", 0.0);
}


double Limelight::IsTargetFound()
{
	return tv;
}

double Limelight::GetArea()
{

	return area;
}

double Limelight::GetXOffset()
{
	return X;
}

double Limelight::GetYOffset()
{
	return Y;
}
double Limelight::GetHeight()
{
	return Height;
}

double Limelight::GetWidth()
{
	return Width;
}
void Limelight::SetVisionModeOn(bool VisionModeOn){
	Table->PutNumber("ledMode", VisionModeOn ? 0.0 : 1.0);
	Table->PutNumber("camMode", VisionModeOn ? 0.0 : 1.0);
}

