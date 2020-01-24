/*----------------------------------------------------------------------------*/
    2 /* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
    3 /* Open Source Software - may be modified and shared by FRC teams. The code   */
    4 /* must be accompanied by the FIRST BSD license file in the root directory of */
    5 /* the project.                                                               */
    6 /*----------------------------------------------------------------------------*/
    7  
    8 #pragma once
    9  
   10 // clang-format off
   11 #ifdef _MSC_VER
   12 #pragma message "warning: Including this header drastically increases compilation times and is bad style. Include only what you use instead."
   13 #else
   14 #warning "Including this header drastically increases compilation times and is bad style. Include only what you use instead."
   15 #endif
   16  
   17 // clang-format on
   18  
   19 #if __has_include(<cameraserver/CameraServer.h>)
   20 #include <cameraserver/CameraServer.h>
   21 #include <vision/VisionRunner.h>
   22 #endif
   23  
   24 #include "frc/ADXL345_I2C.h"
   25 #include "frc/ADXL345_SPI.h"
   26 #include "frc/ADXL362.h"
   27 #include "frc/ADXRS450_Gyro.h"
   28 #include "frc/AnalogAccelerometer.h"
   29 #include "frc/AnalogGyro.h"
   30 #include "frc/AnalogInput.h"
   31 #include "frc/AnalogOutput.h"
   32 #include "frc/AnalogPotentiometer.h"
   33 #include "frc/AnalogTrigger.h"
   34 #include "frc/AnalogTriggerOutput.h"
   35 #include "frc/BuiltInAccelerometer.h"
   36 #include "frc/Compressor.h"
   37 #include "frc/Counter.h"
   38 #include "frc/DMC60.h"
   39 #include "frc/DigitalInput.h"
   40 #include "frc/DigitalOutput.h"
   41 #include "frc/DigitalSource.h"
   42 #include "frc/DoubleSolenoid.h"
   43 #include "frc/DriverStation.h"
   44 #include "frc/Encoder.h"
   45 #include "frc/ErrorBase.h"
   46 #include "frc/GearTooth.h"
   47 #include "frc/GenericHID.h"
   48 #include "frc/I2C.h"
   49 #include "frc/InterruptableSensorBase.h"
   50 #include "frc/IterativeRobot.h"
   51 #include "frc/Jaguar.h"
   52 #include "frc/Joystick.h"
   53 #include "frc/NidecBrushless.h"
   54 #include "frc/Notifier.h"
   55 #include "frc/PIDController.h"
   56 #include "frc/PIDOutput.h"
   57 #include "frc/PIDSource.h"
   58 #include "frc/PWM.h"
   59 #include "frc/PWMSpeedController.h"
   60 #include "frc/PWMTalonSRX.h"
   61 #include "frc/PWMVictorSPX.h"
   62 #include "frc/PowerDistributionPanel.h"
   63 #include "frc/Preferences.h"
   64 #include "frc/Relay.h"
   65 #include "frc/RobotBase.h"
   66 #include "frc/RobotController.h"
   67 #include "frc/RobotDrive.h"
   68 #include "frc/SD540.h"
   69 #include "frc/SPI.h"
   70 #include "frc/SensorUtil.h"
   71 #include "frc/SerialPort.h"
   72 #include "frc/Servo.h"
   73 #include "frc/Solenoid.h"
   74 #include "frc/Spark.h"
   75 #include "frc/SpeedController.h"
   76 #include "frc/SpeedControllerGroup.h"
   77 #include "frc/Talon.h"
   78 #include "frc/Threads.h"
   79 #include "frc/TimedRobot.h"
   80 #include "frc/Timer.h"
   81 #include "frc/Ultrasonic.h"
   82 #include "frc/Utility.h"
   83 #include "frc/Victor.h"
   84 #include "frc/VictorSP.h"
   85 #include "frc/WPIErrors.h"
   86 #include "frc/XboxController.h"
   87 #if __has_include("frc/buttons/InternalButton.h")
   88 #include "frc/buttons/InternalButton.h"
   89 #include "frc/buttons/JoystickButton.h"
   90 #include "frc/buttons/NetworkButton.h"
   91 #include "frc/commands/Command.h"
   92 #include "frc/commands/CommandGroup.h"
   93 #include "frc/commands/PIDCommand.h"
   94 #include "frc/commands/PIDSubsystem.h"
   95 #include "frc/commands/PrintCommand.h"
   96 #include "frc/commands/StartCommand.h"
   97 #include "frc/commands/Subsystem.h"
   98 #include "frc/commands/WaitCommand.h"
   99 #include "frc/commands/WaitForChildren.h"
  100 #include "frc/commands/WaitUntilCommand.h"
  101 #endif
  102 #include "frc/drive/DifferentialDrive.h"
  103 #include "frc/drive/KilloughDrive.h"
  104 #include "frc/drive/MecanumDrive.h"
  105 #include "frc/filters/LinearDigitalFilter.h"
  106 #include "frc/interfaces/Accelerometer.h"
  107 #include "frc/interfaces/Gyro.h"
  108 #include "frc/interfaces/Potentiometer.h"
  109 #include "frc/smartdashboard/SendableChooser.h"
  110 #include "frc/smartdashboard/SmartDashboard.h"
wpilibc
src
main
native
include
frc
WPILib.h