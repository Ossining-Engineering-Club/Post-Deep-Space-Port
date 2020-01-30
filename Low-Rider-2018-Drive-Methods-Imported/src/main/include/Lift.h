
#include <frc/Encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/Phoenix.h>

class Lift{
    private:
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX *LiftBackMotor;
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX *LiftFrontMotor;
        frc::Encoder *liftEncoder;
        frc::SmartDashboard *smartDash;
    public:
        Lift(frc::SmartDashboard *dash);
        void SetToPosition(double power, double EncoderPosition);
        void SetPower(double power);
        double GetEncoderPosition();
};