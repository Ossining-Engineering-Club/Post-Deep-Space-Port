#pragma once
#define VERSION "SPECIAL AUTO TESTING 1.0"

// drive constants
#define LEFTMULT	1.0	//was  1.05 multiplier for the left drive relative to the right

// Sensor constants
#define AUTOGYROCONST 0.06			// was .07273 // was 0.016
#define ANGTOLERANCE 5.0
#define ENCODERCONST 0.0649026

//PID Constants
#define TIMEPERIOD 0.00001
#define PCONSTANT -0.006
#define ICONSTANT -0.025	// was 300
#define DCONSTANT -0		// was 0.000125


// VISION
#define MINVISION 168
#define MAXVISION 198
#define VISIONTARGET 178
#define VISIONMAXHEIGHT 70.0
#define XDIFFERENCEVAL 60
#define AUTODRIVEVISION 0.3

#define AUTOVISIONSPEED	0.36		// check this value!!!
#define AUTOTIMEMAX 15.0

// for camera values
#define XRESOLUTION	320
#define YRESOLUTION 240
#define BRIGHTNESS 20
#define EXPOSURE 10
#define WBM 0
#define FPS 60
#define LED_VALUE 1.0

//Arm:
#define ARM_PROP 0.05
#define ARM_ACCURACY 0.5
#define ARM_MANUAL_THRES 0.1
#define ARM_PICKUP_POS -84.0
#define ARM_HOLD_POS -70.0
#define ARM_LEV_2_POS -29.0
#define ARM_LEV_3_POS -17.2
#define ARM_CARGO1_POS -75.0

//Lift:
#define LIFT_PROP 0.01
#define LIFT_ACCURACY 10.0
#define LIFT_PICKUP_POS 0.0
#define LIFT_HOLD_POS 0.0
#define LIFT_LEV_2_POS 407.75
#define LIFT_LEV_3_POS 830.0
#define LIFT_CARGO1_POS 0.0
#define MINLIFT 65.0 //Was 40.0
#define MAXLIFT 920.0 //975.5 is the abolute top
