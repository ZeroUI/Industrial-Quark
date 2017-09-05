/*
		Quark.h
		Copyright (c) 2016 ZeroUI. All right reserved.
		Author : Jali, Nikhil, July, 2016
		E-mail : nikhil@zeroui.com

		This library is free software; you can redistribute it and/or
		modify it under the terms of the ?????

		This library is distributed in the hope that it will be useful,
		but WITHOUT ANY WARRANTY; without even the implied warranty of
		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
		Lesser General Public License for more details.

		Quark Servo Manual 
		===================================================================
		
		====================================================================



		====================================================================
		Quark Pin Description See. Manual p17)

		====================================================================


		====================================================================
		NOTICE!

	

		====================================================================
*/
// #define DEBUG_MODE				// Uncomment to show Quark Debug Messages

#ifndef IndustrialQuark_h
#define IndustrialQuark_h
#include "Arduino.h"
#include <functional>

#define QRK_EXEC_RATE 20// Execution Rate in [msec]

// Define External Pin Connections
#define AN0 102
#define AN1 103
#define D0 12
#define D1 14
#define D2 0
#define D3 3 
#define D4 16
#define D5 1

// System Modes
#define QUninit	 0		// Quark is uninitialized
#define QExecOff 1		// Quark Motor Control is OFF.
#define QExecOn	 2		// Quark Drive is ON
#define QError	 3		// Quark is in ERROR State

// Motor Modes
#define Mode_COAST	0	
#define Mode_BRAKE	1
#define Mode_ANGLE	2
#define Mode_RPM	3

// Direction
#define Direction_STOP 	0
#define Direction_CW 	1
#define Direction_CCW	-1

// Function Handler Definition
typedef std::function<void(float Current)> ERR_CUR_HANDLER;	// 1-Seeking permission

// Class and method definition
class cQuark
{
public: 
	cQuark();
	static volatile float P_rpm,I_rpm,D_rpm;					// PID Gain parameters for RPM
	static volatile float P_cur,I_cur,D_cur;					// Gains for Current
	
	static volatile float P_ang,I_ang,D_ang;					// PID Gain parameters for Angle
	static volatile int PIDOutSat_ang,PIDOutSat_rpm;			// Clamp PIDOutSat-PID output Clamp (0-100)%
	static volatile float IsumMax_ang,IsumMax_rpm;				// IsumMax-Limits Integral gain wind-up[0-1000].
	static volatile float IsumKb_ang,IsumKb_rpm;				// IsumKb-Back calculation to discharge Iwindup. 0-disabed 1-discharge fast
	static volatile int PIDOut;									// Indicates output value of PID Output (-PIDOutSat to +PIDOutSat)
	static volatile unsigned int PwmFreq;						// Set Motor Frequency (Hz)
	static volatile float tolAngle,tolRpm;						// acceptable tolerance on Angle and Rpm (deg, rpm)
	static volatile unsigned int ExecutionRate;					// Set Sensor sampling and PID execution rate [msec]
	static volatile float CurrentLim;							// Sets the Max current when exceeded for CurrentLimTrigTime triggers an error [A]
	static volatile float dCurrentLim;							// Sets the Max current Rate of Change that triggers an error [A/sec]
	static volatile time_t CurrentLimTrigTime;					// Set time for triggering current limit error [msec]
	static volatile float RpmFilter,AngleFilter,Currentfilter;	// Set proportional filter values [0-1]: 0-don't change    1-change very fast
	static volatile float RpmPIDFilter;							// Sets the filter for PID control on the RPM Control Out.
	static volatile float rpm,angle,current;					// These variables can be used to directly read speed(rpm), angle(degrees -160 to 160) and current(A)
	static volatile float vbatt;								// This is the battery voltage for external reading

	// Initialize and Handling Functions
	static void init(); 										// Initializes Quark with execution rate in [msec]
	static void handle();										// Include this in main loop

	// Motor Operations for USER (CALL THESE)
	static void driveOn();           					  		// Turns drive on while keeping motor in coast 
	static void driveOff();          					  		// Disables Motor Drive and Task Execution. Saves power and execution
	static void brake();             							// Applies electronic brakes to slow motor
	static void stop();											// Ensures the motor comes to a stop. Then puts it in coast mode. Once triggered, it is completed before any other operation.
	static void coast();										// Motor is de-energized in a free-wheeling mode
	static void ResetError();									// Clears Motor related Errors

	// Set Motor Motion Parameters (CALL THESE)
	static bool setRpm(int Rpm);     							// -90 rpm < RPM   < 90 rpm
	static bool setAngle(int Angle);    						// -160deg < Angle < 160deg
	
	//setAngle emergency stop functions
	static void setrunEStopLED(int run);
	static int getrunEStopLED();

	static void setPwmFreq(unsigned int Freq);
	void setCurrentErrorCb(ERR_CUR_HANDLER cb);					// Set Callback for Current Error

	// To Read Operation Parameters (CALL THESE)
	static int getState();         								// Get Quark State: 0-Not Initialized, 1-ExecutionOff, 2-ExecutionOn, 3-Error
	static int getMode();										// Get Motor Mode: Mode_ {COAST, BRAKE, ANGLE, RPM}
	static float getRpm();										// Returns motor rpm (updated every ExecutionTime)
	static int getAngle();										// Returns motor angle (updated every ExecutionTime)
	static float getCurrent();									// Returns motor current (updated every ExecutionTime)
	
	// To Read Analog Inputs
	static int analogRead(uint8_t pin);

	// Motor Specificy Estimation and Control Algorithms
	static void updateAngleRpmCur();							// Updates Angle and Rpm of the Motor
	static void motorControlTask();								// Manages Motor Driving
	static int motorRpmPID();									// RPM PID Control Loop
	static int motorAngPID();									// Angle PID Control Loop

	// Internally monitored and used Variables
	static volatile int kQuarkState;							// 0-Not Init, 1-ExecOff, 2-ExecOn, 3-Error
	static volatile int kMotorMode;								// 0-Coast, 1-Brake, 2-Angle, 3-Rpm
	static volatile int kMotorDirection;						// 0-Not Moving, 1-CW, 2-CCW
	static volatile int kMotorCmdDirection;						// Powered Motor Direction
	static volatile int kRefRpm, kRefAngle;						// Reference rpm/angle specified by user
	static volatile float kRpm, kAngle, kCurrent, kVbatt; 	 	// Internal sampled parameters
	static volatile int kMaxDeadBandCount;						// Maximum expected execution cycles while in dead-band for 5rpm. Computed in Quark.init()
	static volatile int kMaxCurrentExceedCount;					// Maximum counts of current limit exceed in steps of ExecutionRate after which error is triggered
	static volatile bool kStopCmdIssued;						// Indicates if a stop command has been issued
	static volatile bool kIsSensorInDeadBand;					// Flag to indicate if angle is in stop band
	static volatile bool kIsSensorInTrustBand;					// Flag to indicate if RPM is being actively updated

	//setAngle emergency stop variables
	static volatile int rollingSize;
	static volatile float rollingCurrents[20];
	static volatile int oldestCurrent;
	static volatile bool runEStopLED;
	
	static ERR_CUR_HANDLER _err_cur_Cb;							// Variable to store Over-Current Handler
};  //ICACHE_RAM_ATTR

extern cQuark Quark;
#endif

// Additional DEBUG Functions
#ifdef DEBUG_MODE
#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#define DEBUG_NUM(...) Serial.print( __VA_ARGS__ )
#endif
#else
#define DEBUG_MSG(...)
#define DEBUG_NUM(...)
#endif

