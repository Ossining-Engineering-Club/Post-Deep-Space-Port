#pragma once
#include <frc/Joystick.h>
#include <frc/buttons/JoystickButton.h>


class OECJoystick{
    private:
        frc::Joystick *stick;
        frc::JoystickButton *buttons[11];
    public:
        OECJoystick(int port);
        double GetX();
        double GetY();
        double GetZ();
        bool GetTrigger();
        bool GetButton(int buttonNum);
};