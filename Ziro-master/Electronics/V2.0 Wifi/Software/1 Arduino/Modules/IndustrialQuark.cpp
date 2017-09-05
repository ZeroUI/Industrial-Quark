#include "Arduino.h"
#include "IndustrialQuark.h"

//Ansh

// Hardware Pins
#define S0 15
#define S1 2
#define S2 4
#define AIsns 101
#define APos  100
#define MOT_1 5
#define MOT_2 13

// Defining Null 
#define nil 2325



cQuark Quark;

/******************************** Default Variable Declaration ******************************/
volatile float cQuark::P_rpm, cQuark::I_rpm, cQuark::D_rpm;		// RPM PID gain values
volatile float cQuark::P_cur, cQuark::I_cur, cQuark::D_cur;		// RPM current PID gain values
volatile float cQuark::P_ang, cQuark::I_ang, cQuark::D_ang;		// ANGLE PID gain values
volatile float cQuark::IsumMax_rpm,cQuark::IsumMax_ang;			// Limit max integral sum
volatile float cQuark::IsumKb_rpm,cQuark::IsumKb_ang;			// Integral Anti-windup Parameters IsumKb=[0,1]
volatile int cQuark::PIDOutSat_rpm, cQuark::PIDOutSat_ang;		// PID Max Clamp
volatile int cQuark::PIDOut;									// Output of PID to Motor
volatile float cQuark::CurrentLim;								// Current limit in [A]
volatile float cQuark::dCurrentLim;								// Rate of Change of Current Lim [A/sec]
volatile time_t cQuark::CurrentLimTrigTime;						// Current Limit Exceed Error Trigger time [msec]
volatile float cQuark::tolAngle,cQuark::tolRpm;					// Tolerance on Angle[deg] and Speed[rpm]
volatile unsigned int cQuark::PwmFreq;							// PWM Output Frequency [Hz]

// Angle, Speed and Current Sensor Related
volatile float cQuark::RpmFilter;								// Proportional Filter Constant (0 to 1) See Below
volatile float cQuark::AngleFilter;								// 0- Sensor value doesn't update
volatile float cQuark::Currentfilter;							// 1- Updates too rapidly and is noisy
volatile float cQuark::RpmPIDFilter; 							// Filter for Controlling rate of change of RPM
volatile float cQuark::rpm,cQuark::angle,cQuark::current;		// For direct access. Eg: "Serial.print(Quark.rpm)"
volatile float cQuark::vbatt;
volatile bool cQuark::kIsSensorInDeadBand;						// Indicates if Position Sensor is in DeadBand
volatile bool cQuark::kIsSensorInTrustBand;						// Indicates if RPM is actively being updated

// Task Execution Related
volatile int cQuark::kQuarkState;								// Keep Track of Overall Mode [Private]
volatile int cQuark::kMotorMode;								// Keep track of motor mode [Private]

// Internal Variables (Private)
volatile int cQuark::kMotorCmdDirection;						// Commanded motor direction
volatile int cQuark::kRefRpm, cQuark::kRefAngle;				// Reference set rpm/angle
volatile float cQuark::kRpm, cQuark::kAngle, cQuark::kCurrent;	// Internal sensor values
volatile bool cQuark::kStopCmdIssued;							// Status of Stop command
volatile float cQuark::kVbatt;									// BATTERY VOLTAGE [Volts]. NEEDS HARDWARE/SOFTWARE FIX

ERR_CUR_HANDLER cQuark::_err_cur_Cb;							// Function Handler for Current Errors

//setAngle emergency stop variables
volatile int cQuark::rollingSize = 20;
volatile float cQuark::rollingCurrents[20];
volatile int cQuark::oldestCurrent;
volatile bool cQuark::runEStopLED;

/******************************** cQuark State Function Declaration ******************************/
// Quark State, 	kQuarkState: 0-Not Initialized, 1-QuarkExecOff, 2-QuarkExecOn, 4-Error
// 0-Not Initialized	Library included but Init not enabled
// 1-QuarkExecOff 		No sensors are sampled(to save power) and motor is in coast state
// 2-QuarkExecOn		Sensors are sampled as per exec time
// 3-Error				Some error is preventing the motor from running
//
// MotorState:	kMotorMode UsedBy
// 0-Coast		(Drive Off, Coast, Init, Stop*
// 1-Brake		(DriveOn,	Brake, Stop*
// 2-Angle		(setAngle)
// 3-Rpm		(setRpm)
// *Stop: Stop uses brake mode and brings device to a stop and then turns it to coast mode


// Class Initialization
cQuark::cQuark() // Initialize Class Variables
{
	// RPM PID gain values
	cQuark::P_rpm=40;					
	cQuark::I_rpm=20.00; 
	cQuark::D_rpm=505.00;					
	cQuark::IsumKb_rpm=4.00;			// Integral Anti-windup Parameters IsumKb=[0,1]
	cQuark::IsumMax_rpm=30;				// Limit max integral sum
	cQuark::PIDOutSat_rpm=60; 			// PID Max Clamp

	// RPM Based Current PID Control
	cQuark::P_cur=5.00;
	cQuark::I_cur=0.70;
	cQuark::D_cur=0.00;					// Keep this control predominantly a PI controller
	cQuark::RpmPIDFilter=0.450;		// This is part of the adaptive filter in the current control loop for the RPM

	// ANGLE PID gain values
	cQuark::P_ang=1.50;					//Ansh - PID vals for Quark 1
	cQuark::I_ang=0.45;
	cQuark::D_ang=0.20;					
	cQuark::IsumKb_ang=0;				// Integral Anti-windup Parameters IsumKb=[0,1]
	cQuark::IsumMax_ang=100;			// Limit max integral sum
	cQuark::PIDOutSat_ang=100; 			// PID Max Clamp

	cQuark::PIDOut=0;					// Output of the overall PID

	// Current Related
	cQuark::dCurrentLim=30;				// Limit on Rate of Change of Current Rise
	cQuark::CurrentLim=3750;			// Current limit in [mA]
	cQuark::CurrentLimTrigTime=100;		// Current Limit Exceed Error Trigger time [msec]

	cQuark::tolAngle=1.5;
	cQuark::tolRpm=1;					// Tolerance on Angle[deg] and Speed[rpm]
	cQuark::PwmFreq=9000;				// PWM Output Frequency [Hz]

	// Angle, Speed and Current Sensor Related
	cQuark::RpmFilter=0.020;			// Proportional Filter Constant (0 to 1) See Below
	cQuark::AngleFilter=0.45;			// 0- Sensor value doesn't update
	cQuark::Currentfilter=0.05;			// 1- Updates too rapidly and is noisy

	// Internal Variables (Private)
	kMotorCmdDirection=Direction_STOP;	// Commanded motor direction
	kRefRpm=0;							// Reference set Rpm 
	kRefAngle=0;		 				// Reference set Angle
	kStopCmdIssued=0;					// Status of Stop command

	cQuark::_err_cur_Cb=0;

	//setAngle emergency stop variables initializations
	cQuark::rollingSize = 20;
	for (int i = 0; i < rollingSize; i++) {
		cQuark::rollingCurrents[i] = 0;
	}
	cQuark::oldestCurrent = 0;
	cQuark::runEStopLED = 0;

	DEBUG_MSG("\nQuark Initialized\n");
}


/********************************************** Initialization Function ***************************************/
// This function sets up the Hardware Pins, PWM Frequency, Computes variables for safety checks
void cQuark::init()
{
	// Set Hardware Modes
	pinMode(MOT_2,OUTPUT);
	pinMode(MOT_1,OUTPUT);
	pinMode(S0,OUTPUT);
	pinMode(S1,OUTPUT);
	pinMode(S2,OUTPUT);
	pinMode(A0,INPUT);

	// Set PWM Frequency
	cQuark::setPwmFreq(PwmFreq);									// Set PWM Frequency

	// Sets the Quark and Motor Mode
	kQuarkState=QExecOn;											// Initialize Quark Mode to On
	kMotorMode=Mode_COAST;											// Set Motor Mode to None
	cQuark::coast();
}


/**** This function is executed every QRK_EXEC_RATE milliseconds****/
// static void QuarkStateMachine() ICACHE_RAM_ATTR;
void cQuark::handle() 
{	
	static time_t LastCheckTime=0;

	if(time_t(millis()-LastCheckTime)>QRK_EXEC_RATE)
	{
		cQuark::updateAngleRpmCur();		// Reads angle sensor, determines position, estimates direction and speed

		cQuark::motorControlTask();		// Controls the motor based on user set-points

		LastCheckTime=millis();
	}
}


// This Interfaces with the On-board Mux to read the 4 Analog Signals i.e. AN0, AN1, APos and AIsns
int cQuark::analogRead(uint8_t pin)
{
	if(pin>=100 && pin<=104)
	{	
		digitalWrite(S2,((pin-100)&0b100?HIGH:LOW));
		digitalWrite(S1,((pin-100)&0b010?HIGH:LOW));
		digitalWrite(S0,((pin-100)&0b001?HIGH:LOW));
		delayMicroseconds(4);
		int AdcVal=0;
		AdcVal=::analogRead(A0);
		digitalWrite(S2,HIGH);			// To Reduce Power Consumption on ESP LED
		return AdcVal;
	}
	else 
	{
		digitalWrite(S2,HIGH);			// To reduce power consumption on ESP LED
		return digitalRead(pin)*1023;
	}
} 


// This function updates the position, current and rpm of quark
/* GENERAL NOTE: 
Kindly call Nikhil Jali (+1-765-476-3370) to fully understand the RPM calculation algorithm or consider switching to a pulse encoder based method
He may have forgotten it, but is sure to recall it as he wrote this piece of complicated code...that works!!!

NOTE: 
Angle Estimation:
	Angle is estimated between the deadBand limits. The angle is limited to ±160° beyong the deadAngleTol which is buffer.

Rpm Estimation: 
	Rpm estimation is dependent upon the angle estimation. Furthermore buffer is added to the Angle values for proper estimation.
	Rpm is estimated between [-rpmCalc_trustAngle° and +rpmCalc_trusAngle°]. Recommended Values(RecVal): [-95° to 95°]

Dead Band Computations:
	During the dead-band, the algorithm computes the maximum expected time that the sensor must spend in the dead-band
	i.e. deadBandPredictedExitDuration. This algorithm allows for 10rpm reduction in speed while in the dead-band

	If the position sensor does not leave the dead-band within this time, the RPM value is discharged to zero.


Total Regions: 
****ACTIVE REGION- Position can more or less be inferred in this region
		-Angle Update Region:  	Guaranteed sensor position. Mechanical position is guaranteed between ±160°. Electrical goes to ±170°
								AngleUpdateBand=[deadBandMin+deadAngleTol   to  deadBandMax-deadAngleTol] in ADC counts

		-RPM Trust Region: 	This region is dynamically computed given RPM. The trust region decrees as rpm increases. This is due to the 
							sampling rate and nature of the sensing system given the capacitances.
							RpmUpdateBand=  [-rpmCalc_trustAngle to  rpmCalc_trustAngle ] in degrees


****DEAD BAND- 	Only rpm can be vaguely estimated here. Angle could potentially be found using RPM, but no accurate position control is possible
				Deadband=[0 to deadBandMin+deadBandTol] and [deadBandMax-deadBandTol to 1024	] in ADC counts
*/
void cQuark::updateAngleRpmCur()
{
	// Timing Related
	static time_t currentTime=-1;
	currentTime=millis();

	// Angle Estimation Variables
	const int maxAngleValue=160;						// RecVal: 160. Maximum guaranteed Angle Value
	const float minAngleChange=0.3;						// RecVal: 0.3. Minimum change in Angle to update value
	static int rawAngSensor=0;							// Direct Angle Sensor Value
	static time_t lastAngleSampleTime=0,dt;				// To keep track of sample and elapsed time
	static float rawAngle=0,lastAngle=0;				// Raw Angle and previous smoothened Angle
	static float dAngle;								// Rate of change in angle
	static float prev_dAngle=0;

	// Rpm Estimation Variables
	static time_t lastRpmSampleTime=0;					// Time at which RPM was last sampled
	static float rawRpm=0;								// Variable to compute raw Rpm value from dAngle
	static int dRpmMax=20;								// Limit max change of RPM estimation per measurement cycle
	const int rpmCalc_minDAngle=0.5;					// RecVal:1.5 Minimum change of angle requried for RPM estimation. Set [rpmCalc_minDAngle > minAngleChange]
	const int rpmCalc_maxDAngle=15;						// RecVal:15. Maximum change of angle beyond which angle change considered false. 7deg
	
	// Trust Angle
	static float rpmCalc_trustAngle=0;					// Angle within which to update RPM. This is automatically found based on RPM
	static float rpmCalc_NonTrustAngle=0;				// Angle outside DeadZone and Trust Angle. Found using: 360-(2*rpmCalc_trustAngle)
	static float rpmCalc_NonTrustAngle_atExit=0;		// Last Updated NonTrust Angle at exit of Trust Band
	static float rpmCalc_predictedNonTrustTime=0;		// Expected time spent outside Trust Angle and Dead-Zone [msec]
	static float rpmCalc_NonTrustTime=0;				// Actual time spent in Non Trust Region
	static float rpmCalc_trustBandCurrent=0;			// Current in Trust Angle

	// Direction Estimation Variables
	static float estDirection=0;						// Estimated direction of Motion. 
	const float estAlpha=0.6;							// These values are for trusting estimated dAngle movements. 
														// Note: estAlpha+NestAlpha=1

	// Dead band Related Variable
	static int deadBandCount=0;							// Keeps count of cycles in dead-band
	static bool DeadBandExit=0;							// Keeps track of whether position exited from Dead Band
	static time_t deadBandEntryTime=-1;					// Keeps track of time of first entry into dead-band in a given cycle
	static int deadBandPredictedExitDuration=0;			// Computes the maximum time to be spent in dead-band

	// Current Estimation Variables
	static float prevCurrent=0,rawCurrent=0;			// Previous Current value and Raw Current
	static float currentExceedCount=0;					// Number of exec cycles where kcurrent>Limit
	static int rawCurrentSensor=0;						// The value of the raw current sensor


	// DEAD BAND LIMIT ON ANGLE SENSOR
	// The angle estimation algorithm uses a buffer zone between the actual sensor dead-zone
	// Calibration Note: 
	// 1. Set [deadBandMin,deadBandMax] exactly at very edge where the sensor and readings flip, stop, or are invalid
	// 2. Angle computation is done before 20 ADC counts of deadBand
	const int deadBandMax=500;		// RecVal: 500 	=>Set this to the max value read on the Potentiometer. 
	const int deadBandMin=50;		// RecVal: 50	=>Set this to the min value read on the Potentiometer
	const int deadBandTol=10;		// RecVal: 10	=>Set to allow for ample sensing space at high speeds
	const int deadAngleTol=20;		// RecVal: 20	=>Set a few counts above deadBandTol


	/******************************************** Position Sensor Sampling *****************************************************/
	rawAngSensor=cQuark::analogRead(APos);			// Read Position Sensor
	dt=(micros()-lastAngleSampleTime);				// Find change in sample time [usec]
	time_t templastAngleSampleTime;
	templastAngleSampleTime=micros()-3;				// Record angle sampled time


	/********************************************* Angle Update Region ********************************************************/
	// Check if Sensor is within Limits of Safe Band
	if(rawAngSensor>deadBandMin+deadBandTol && rawAngSensor<deadBandMax-deadBandTol)  // Ensure sensor is well within working band
	{
		// Check if the sensor just exited the deadBand
		if(deadBandCount>2)			// DeadBandCount implies sensor just exited from deadband
			DeadBandExit=1;			// Set Flag. Sensor just exited deadband
		else
			DeadBandExit=0;			// Clear Flag
		kIsSensorInDeadBand=0;		// Clear Flag

		/************************************* ANGLE ESTIMATION *************************************/
		// Check if Position Sensor just exited the deadband
		if(!DeadBandExit)
		{
			// Convert ADC to rawAngle degrees
			// Below Equation obtained on curve fitting and manual testing
			rawAngle=(-0.642*rawAngSensor +184.25);
			kAngle=AngleFilter*rawAngle + (1-AngleFilter)*kAngle;	// Apply smoothing filter and update kAngle
		}
		// Check if position sensor just exited Dead Band
		else if(DeadBandExit)
		{
			if(rawAngSensor<deadBandMin+deadAngleTol 		&& DeadBandExit)			// Angle Sensor at Limits			
			{
				rawAngle= 160;
				kAngle  = 160;
			}

			else if(rawAngSensor>deadBandMax-deadAngleTol 	&& DeadBandExit)			// Angle Sensor at Limit
			{
				rawAngle= -160;
				kAngle  = -160;
			}
		}
		// Note: Angle computation is done only after stabilized readings after deadband exit.
		// Having buffer deadBangAngleTol allows for sure and stable readings after a dead-band exit


		// Constraining the Overall Angle Values
		kAngle=constrain(kAngle,-maxAngleValue,maxAngleValue);				// Limit the angle values

		// Find rate of Changes
		dAngle=(kAngle-lastAngle);						// Find change in angle [deg]
		
		// Update Angle only if change was significant
		if(abs(dAngle)>minAngleChange)							
		{
			lastAngle=kAngle;
			cQuark::angle=kAngle;						// update public variable
		}

		// Reset Dead-band Variables
		DeadBandExit=0;
		deadBandCount=0;								// Reset Dead Band Counter
	}
	// If in Dead-Band increment deadBandCount
	else 
	{
		// Record time during first entry into dead-band
		if(deadBandCount==0)
		{
			// Compute First Time of First Entry Into DeadBand
			deadBandEntryTime=currentTime;			
		}

		// Reset Angle Variables
		if(deadBandCount>3)
		{
			if(rawAngSensor<=deadBandMin+30)
				kAngle=160;
			else if(rawAngSensor>=deadBandMax-30)
				kAngle=-160;

			angle=kAngle;
		}

		kIsSensorInDeadBand=1;									// Set Stop band Status
		kIsSensorInTrustBand=0;

		deadBandCount++;									// Count Up Dead band
		deadBandCount=constrain(deadBandCount,0,1000);		// 500 corresponds to expected dead time rpm for 1sec
	}


	/********************************************* RPM Calculation ************************************************************/
	// Trust Angle Calculations 
	rpmCalc_trustAngle=constrain((-0.65*abs(kRpm))+110,40,110);	// Experimentally found eqn.  [Higher the RPM->Smaller the Trust Angle]
	rpmCalc_NonTrustAngle=360-(2*rpmCalc_trustAngle);			// Find angle span that cannot be trusted
	kIsSensorInTrustBand=0;										// Clear Trust Band Flag

	// Trust Angle Calculations
	rpmCalc_predictedNonTrustTime=(float)(rpmCalc_NonTrustAngle/((abs(rpm)+1)*6))*1000;		// Expected time outside trust zones [msec]
	rpmCalc_predictedNonTrustTime=constrain(rpmCalc_predictedNonTrustTime,100,8000)+100;	// Constrain and add speed variation buffer 

	/*************************************** Inside Trust Region *************************************/
	if(abs(angle)<rpmCalc_trustAngle && !kIsSensorInDeadBand)
	{		
		// If Change in angle is substantial, then compute RPM
		if(abs(dAngle)>rpmCalc_minDAngle && abs(dAngle)<rpmCalc_maxDAngle)
		{
			// Compute Raw Rpm
			if(time_t(currentTime-lastAngleSampleTime)<30)			// Ensure Sampling Rate of RPM is maintained
			{
				rawRpm=(float)(((1000*dAngle)/dt)*1000);			// Find rate of change of angle: deg/sec
				rawRpm=rawRpm/6;									// Convert degree per second to rpm
			}
			else 
				rawRpm=kRpm;										// If RPM sampling interrupted, do not change RPM

			// Filter Raw RPM
			rawRpm=RpmFilter*(rawRpm) + (1- RpmFilter)*kRpm;	// Apply Aggressive Smoothing filter to it.
			kRpm=constrain(rawRpm,kRpm-dRpmMax,kRpm+dRpmMax);	// Limit rate of change of est. RPM per execution cycle	
		}

		// If Amount of Angle Change is insufficient, discharge RPM value
		else if(abs(dAngle)<rpmCalc_minDAngle)
		{
			rawRpm=RpmFilter*(0.33*kRpm) + (1- RpmFilter)*kRpm;	// Apply Aggressive Smoothing filter to it.
			kRpm=constrain(rawRpm,kRpm-dRpmMax,kRpm+dRpmMax);	// Allow for Quick Discharge
		}

		//	Trust Angle Based Computations
		rpmCalc_trustBandCurrent=kCurrent;					// Capture Current in Trust Region
		kIsSensorInTrustBand=1;								// Enable Trust Region Flag
		rpmCalc_NonTrustTime=0;								// Reset Time in non-Trust Angle
		rpmCalc_NonTrustAngle_atExit=rpmCalc_trustAngle;	// Update Non Trust Angle

		// Update Last Rpm Sampled Time
		lastRpmSampleTime=currentTime;					
	}

	/*************************************** Outside Trust Region *************************************/
	else
	{
		// Compute Time Spent in non-Trust Region
		rpmCalc_NonTrustTime=constrain(currentTime-lastRpmSampleTime,1,10000);	// Prevent Divide by 0 error and constrain Max

		// If true RPM value not updated while inside Active region but outside dead-zone, then discharge RPM
		// Discharge RPM if time since update exceeds rpmCalc_NonTrustTime
		if(rpmCalc_NonTrustTime>rpmCalc_predictedNonTrustTime)
		{
			rawRpm=rpmCalc_NonTrustAngle_atExit*1000/(6*rpmCalc_NonTrustTime);
			rawRpm=(RpmFilter)*(rawRpm) + (1-RpmFilter)*kRpm;	// Smoothly change values filter to it.
			kRpm=constrain(rawRpm,kRpm-dRpmMax,kRpm+dRpmMax);	// Allow for Discharge
		}	
	}

	// Safety Catch
	if( time_t(currentTime-lastRpmSampleTime)>10000)
	{
		DEBUG_MSG("\n*RPM CLEARED*\n");
		kRpm=0;
		estDirection=0;
	}

	// Update Public Variables
	if(abs(kRpm)<1.5)
		rpm=0;
	else
		rpm=RpmFilter*rpm + (1-RpmFilter)*kRpm;

	// Record Last Sampled Time
	lastAngleSampleTime=templastAngleSampleTime;




	/******************************************** Current Sensor *************************************************/
	static time_t currentFaultTime=-1;				// This is to capture the fault's first occurence
	static time_t currentLastSampleTime=-1;			// This is to capture the current sensor's last done time
	static int idleCurrent=36;						// Idle Motor Current

	rawCurrentSensor=cQuark::analogRead(AIsns);

	currentTime=millis();
	rawCurrent=constrain((4.2*rawCurrentSensor)-idleCurrent,0,4000);	// mA
	kCurrent=Currentfilter*rawCurrent + (1- Currentfilter)*kCurrent;	// Filter Raw Current

	// Detect If Current Exceed Limit
	if( kCurrent > cQuark::CurrentLim)
	{
		// Capture time of first current error time
		if(currentFaultTime==0)
			currentFaultTime=currentTime;
		
		DEBUG_MSG("\nQuark:CURR ERR   ");

		if(time_t(currentTime-currentFaultTime)>=500)//cQuark::CurrentLimTrigTime)				// Current has exceeded for limit time
		{
			// Stop the Motor
			cQuark::stop();
			cQuark::kQuarkState=QError;
			cQuark::kRefRpm=0;

			// Current Error. Call External Function
			if(cQuark::_err_cur_Cb)
				_err_cur_Cb(kCurrent);
		}
	}
	else
	{
		currentFaultTime=0;
	}

	/********* UPDATE VARIABLES ***********/
	cQuark::current=kCurrent;


	/******************************** VOLTAGE SENSOR ****************************************/
	// REVISION NOTE: ADD A VBATT CHANNEL TO QUARK. 
	// THIS CODE AND THE RELATED RPM CONTROL ALGORITHM ONLY WORK IF VBATT IS PROVIDED TO THE MUX
	// FOR ZIRO, THE VBATT CHANNEL IS THROUGH AN0 AND COMES FROM THE BOTTOM BOARD.
	// AS FOR PRODUCTION VERSION OF QUARK, A QUICK FIX IS NEEDED IN SOFTWARE AND HARDWARE

	// Read ADC Values
	int adcTemp[2];
	adcTemp[0]=cQuark::analogRead(AN0);	// Vbatt Voltage

	// Find Raw Battery Voltage
	float vBatt_raw;
	const float Vbatt_filter=0.20;
	vBatt_raw=(float) 0.0124*adcTemp[0];

	// For Backward compatibility
	if(vBatt_raw<5 || vBatt_raw>10)
		vBatt_raw=8.4;

	// Voltage filter
	kVbatt=(float)(1-Vbatt_filter)*kVbatt + Vbatt_filter*vBatt_raw;

	// Update Public variable
	cQuark::vbatt=kVbatt;

	// DEBUGGING INFORMATION
	static int DispCount=0;
	if(DispCount++>10)
	{
		DispCount=0;
		DEBUG_MSG("\nQ:%d",kQuarkState);
		DEBUG_MSG("\tM:%d",kMotorMode);
		DEBUG_MSG("\tEstDir:%s",String(estDirection).c_str());
		DEBUG_MSG("\tAngle:%s",String(kAngle).c_str());
		DEBUG_MSG("\tRPM:%s",String(kRpm).c_str());
		DEBUG_MSG("\tCurr:%s",String(kCurrent).c_str());
	}
}



// This function performs the task of PID control and the STOP action
void cQuark::motorControlTask()
{
	/*********************************** STOP COMMAND PROCESSING ***********************************/
	if(kStopCmdIssued || kQuarkState!=QExecOn)
	{
		DEBUG_MSG("\n!!!Stop CMD!!!\n");

		// PUT MOTOR IN BRAKE MODE
		cQuark::brake();

		// KEEP IN BRAKE MODE TILL IT STOPS 
		if(abs(kRpm)<5)
		{
			// PUT MOTOR TO COAST MODE
			cQuark::coast();

			// Set RefANGLE=Curr Pos, Set SPD=0;
			kRefRpm=0;
			kRefAngle=kAngle;

			// RESET kStopCmdIssued
			kStopCmdIssued=0;
		}
		return;
	}


	/************************************* PID PART **************************************************/
	static float Isum=0;											// This is the integral term
	static float dError,currError=0,prevError=0;					// For tracking error
	static float prevPIDoutput=0;									// Calculated PID values -100% to 100%
	static int pwmPIDoutput=0;										// This is the PWM applied to motor
	static float dt; 												//
	
	// Do PID Only if in Angle or Rpm Set Mode.
	if((kMotorMode==Mode_ANGLE || kMotorMode==Mode_RPM) && kQuarkState==QExecOn)
	{
		

		// Get PID Control Values from Respective Control Loops
		// RPM Mode PID
		if(kMotorMode==Mode_RPM)
		{
			PIDOut=cQuark::motorRpmPID();			
		}
		
		// Angle Mode PID
		else if(kMotorMode==Mode_ANGLE)
			PIDOut=cQuark::motorAngPID();

		// PWM Out Calculation
		pwmPIDoutput=(int)abs(PIDOut)*10.23;								// Calculating Output Duty Cycle
		pwmPIDoutput=constrain(pwmPIDoutput,10,1000);


		// Control motor direction based on Angle
		if(PIDOut>3)
		{
			analogWrite(MOT_1,0);
			digitalWrite(MOT_1,0);
			analogWrite(MOT_2,pwmPIDoutput);
			cQuark::kMotorCmdDirection=Direction_CW;
		}
		else if(PIDOut<-3)
		{
			analogWrite(MOT_2,0);
			digitalWrite(MOT_2,0);
			analogWrite(MOT_1,pwmPIDoutput);
			cQuark::kMotorCmdDirection=Direction_CCW;	
		}
		else
		{
			analogWrite(MOT_1,0);
			analogWrite(MOT_2,0);
			digitalWrite(MOT_1,0);
			digitalWrite(MOT_1,0);
			cQuark::kMotorCmdDirection=Direction_STOP;
		}
	}
	else
	{
		PIDOut=0;
		pwmPIDoutput=0;
	}

	// Record Previous PID output
	prevPIDoutput=PIDOut;
	static int DispCount=0;
	if(DispCount++>10)
	{
		DispCount=0;
		DEBUG_MSG("\tPIDOut:%s",String(PIDOut).c_str());
		DEBUG_MSG("\tPwm:%d",pwmPIDoutput);
	}
}



// Motor Speed PID Loop
// Returns unconstrained PID output between -100 and 100 [PIDOutSat]
// NOTE: CURRENT RPM CONTROL IS LIMITED AND UNRELIABLE AT RPMs BELOW 20RPM. 
// HENCE, THE SPEED CONTROL IS DONE PURELY THROUGH CURRENT CONTROL. 
// THIS ALLOWS FOR A ROBUST RESPONSE AND ENSURES RELIABILITY
int cQuark::motorRpmPID()
{
	// Internal Variables
	static float Isum=0;											// This is the integral term
	static float dError=0,currError=0,prevError=0;					// For tracking error
	static float prevFilteredPIDout=0,prevRawPIDoutput=0;			// Track previous raw and filtered PID outputs			
	static float filteredPIDout=0,rawPIDout=0;						// Calculated PID values -100% to 100%
	static float Rpm_ControlOutput=0;								// Final Control Output from the RPM PID control
	static float dt=0; 												//
	static time_t PIDlastDoneTime=0;								// Time keeping in [s]	
	static float FF_input=0;										// FFinput=f(Reference_RPM). This is the feedforward input dependent upon rpm
	static bool updateCurrentTarget;								// Flag 
	static float currentTarget;										// This value loosely refers to the current value to maintain during dead-band

	// Time Keeping
	if(PIDlastDoneTime==0)
	{
		PIDlastDoneTime=millis();
		return 0;
	}
	dt=(float)(millis()-PIDlastDoneTime)/1000;						// Elapsed Time in [sec]
	PIDlastDoneTime=millis();										// Record Start Instance
	
	// Prevent Divide by zero error in derivative term computation
	if(dt<=0)		
		return 0;
	

	/************************  RPM  PID Calculation     ***************/
	// // FeedForward Calculation
	// int tempRPM=abs(kRefRpm);
	// FF_input=(1.2*kRefRpm)+15;				// Open-Load DC% values computed from testing.
	// FF_input=constrain(FF_input,50,100);	// Constrain overall Feed-forward limits. 	
	// FF_input+=(kRpm<20)*(20-abs(kRpm));		// Add low RPM feed
	// FF_input=((kRefRpm>0)?1:-1)*FF_input;	// Set Feed-forward direction based on test Reference RPM
	// FF_input=0;

	// // Calculate Error Parameters
	// currError=kRefRpm-kRpm;							// Current Rpm Error
	// dError=currError-prevError;						// Calculate change in error

	// // Allow Tolerances on Error
	// if(abs(currError)<tolRpm)						// Check if error within tolerance band
	// 	currError=0;
	// if(abs(dError)<1) 								// Check if Rate of Change of error is small
	// 	currError=prevError;

	// // Integral term
	// Isum=(float)Isum+(currError);					// Basic Integral Computing
	// Isum=(float)(Isum - IsumKb_rpm*constrain(Rpm_ControlOutput-prevRawPIDoutput-FF_input,-Isum,Isum));	// Back calculation for quick discharge
	// Isum=constrain(Isum,-IsumMax_rpm,IsumMax_rpm);	// Overall limiting

	// // Derivative Term
	// dError=(float)((currError-prevError));
	// prevError=currError;

	// // PID Calculation
	// rawPIDout=P_rpm*currError + I_rpm*Isum + D_rpm*dError;	// PID Calculation
	// filteredPIDout=rawPIDout;								// Assigning filtered PIDout to rawPIDout

	// // Record PID Outputs i.e. raw and filtered for Next Iteration
	// prevRawPIDoutput=rawPIDout;
	// prevFilteredPIDout=filteredPIDout;

	// // Add PID and FeedForward Inputs
	// Rpm_ControlOutput=FF_input+filteredPIDout;
	// Rpm_ControlOutput=filteredPIDout;

	// // Add Conditioning
	// Rpm_ControlOutput=constrain(Rpm_ControlOutput,-4000,4000);

	// // Flip Control Output by 180degrees if RefRpm is CCW
	// Rpm_ControlOutput*= (kRefRpm>0)?1:-1;


	/***************************** MOTOR RELATED CALCULATIONS **************/
	static float Pi2_60=0.10471955;		// Equivalent of 2*pi/60
	static float Mot_eff=0.60;			// Combined Switching & Gearbox Efficiency. Typ: 60%
	static float Mot_maxTorq=1.4;		// Max Motor Stall Torque at output shaft in Nm. Include Gear Ratio for calculation
	static float Mot_pwrCoeff=Pi2_60*Mot_maxTorq/Mot_eff*10;	// This is the net co-efficient for all constants. P=(2*pi*N(rpm)*Torque(Nm)/60) / Efficiency


	/****************************** CURRENT CONTROL ************************/
	static float Rpm_Cur_Filtered_Input;
	static float Curr_pid_out, Curr_pid_out_prev;				// 
	static float Curr_pid_filt_out, Curr_pid_filt_raw;			// Current PID filtered Out
	static float Curr_dError=0,Curr_error=0,Curr_prevError=0;	// For tracking error
	static float Curr_target;									// in mA
	static float Curr_error_sum;
	static float Curr_ff_out=0;
	static float Curr_ControlOutput=0;
	static float Curr_Filtered_ControlOutput=0;
	
	// Uncomment below for tuning. Use Kp_rpm to set RPM Control Output
	// Rpm_ControlOutput=P_rpm;

	// Filter Out Noisy Input From RPM PID Input
	Rpm_Cur_Filtered_Input=RpmPIDFilter*(Rpm_ControlOutput) + (1-RpmPIDFilter)*Rpm_Cur_Filtered_Input;

	// Calculate Feed-Forward Current
	Curr_ff_out=0;	// Feed-Forward in mA

	// Calculate Current Target
	Curr_target=(Mot_pwrCoeff * Rpm_Cur_Filtered_Input)/kVbatt;				// mA
	Curr_target=constrain(Curr_target+Curr_ff_out,0,cQuark::CurrentLim);	// Add feedforward gain here

	// Recalculate Current Target (Comment Out Below for Including RPM PID Control)
	int tempRef;
	tempRef=abs(kRefRpm);
	Curr_target= constrain(0.001577*(tempRef*(tempRef*tempRef)) - 0.112*(tempRef*tempRef) + (3.058*tempRef) +50,0,1500);
	//Curr_target= constrain((0.1*(tempRef*tempRef)) - (2*tempRef) +10,0,1500);

	// Calculate Error Parameters
	Curr_error=Curr_target-kCurrent+10;		// Present Error
	Curr_dError=Curr_error-Curr_prevError;	// Change of Error
	Curr_error_sum+=Curr_error;				// Summation of errors
	Curr_error_sum=constrain(Curr_error,-500,500);		// Apply constrain on summation
	Curr_prevError=Curr_error;				// Record Previous Error

	// Calculate Current PID Output
	Curr_pid_out= (P_cur*Curr_error) + (I_cur*Curr_error_sum) + (D_cur*Curr_dError);

	// Logical Conditioning
	Curr_pid_out=constrain(Curr_pid_out,0,cQuark::CurrentLim);	// Ensure always positive as current sensor only reads abs(current)

	// Add Feed-Forward and Constraints
	Curr_ControlOutput=(Curr_pid_out);//+Curr_ff_out);
	Curr_ControlOutput=constrain(Curr_ControlOutput,0,150);	
	Curr_ControlOutput*= ((kRefRpm>0)?1:-1);	

	// Apply Filter On Current Controller Output
	Curr_Filtered_ControlOutput=0.95
	*(RpmPIDFilter*(Curr_ControlOutput) + (1-RpmPIDFilter)*Curr_Filtered_ControlOutput);

	// Debug Information
	Serial.printf("%s,",String(kAngle).c_str());
	Serial.printf("%s,",String(kRefRpm).c_str());
	Serial.printf("%s,",String(kRpm).c_str());
	Serial.printf("%s,",String(Rpm_ControlOutput).c_str());
	Serial.printf("%s,",String(Curr_target).c_str());
	Serial.printf("%s,",String(kCurrent).c_str());
	Serial.printf("%s",String(Curr_ControlOutput).c_str());
	Serial.println("");

	// Return Control Outputs
	return Curr_Filtered_ControlOutput;
}



// Motor Angle PID Loop
// Returns unconstrained PID output between -100 and 100 [PIDOutSat]
int cQuark::motorAngPID()
{
	// Internal Variables
	static float Isum=0;											// This is the integral term
	static float dError=0,currError=0,prevError=0;					// For tracking error
	static float prevPIDoutput=0,angPIDout=0;							// Calculated PID values -100% to 100%
	static float dt=0; 												//
	static time_t PIDlastDoneTime=0;								// Time keeping in [s]	

	// Time Keeping
	if(PIDlastDoneTime==0)
	{
		PIDlastDoneTime=millis();
		return 0;
	}
	dt=(float)(millis()-PIDlastDoneTime)/1000;								// Elapsed Time in [sec]
	PIDlastDoneTime=millis();										// Record Start Instance

	// Current Rpm Error
	currError=kRefAngle-kAngle;											// Current Rpm Error

	// Allow Tolerance on Error
	// NOTE: Consider adding a derivative based rate check while making currError=0
	// This ensures currError is made ZERO only when the motor is stationary
	if(abs(currError)<tolAngle)
		currError=0;

	// Integral term
	Isum=(float)Isum+(currError*dt);										// Basic Integral Computing
	Isum=(float)(Isum - IsumKb_ang*constrain(prevPIDoutput-PIDOutSat_ang,-50,50));	// Back calculation for quick discharge
	Isum=constrain(Isum,-IsumMax_ang,IsumMax_ang);									// Overall limiting

	// Derivative Term
	if(dt>0)
		dError=(float)((currError-prevError)/dt);	
	else
		dError=0;
	prevError=currError;

	// PID Calculation
	prevPIDoutput=P_ang*currError + I_ang*Isum + D_ang*dError;			// PID Calculation
	angPIDout=constrain(prevPIDoutput,-PIDOutSat_ang,PIDOutSat_ang);			// Applying saturation limits

	static int DispCount=0;
	if(DispCount++>10)
	{
		DispCount=0;
		DEBUG_MSG("\tAp:%s",String(P_ang).c_str());
		DEBUG_MSG("\tAi:%s",String(I_ang).c_str());
		DEBUG_MSG("\tAd:%s",String(D_ang).c_str());
		DEBUG_MSG("\tRefAng:%d",kRefAngle);
		DEBUG_MSG("\tCurrErr:%s",String(currError).c_str());
		DEBUG_MSG("\tIsum:%s",String(Isum).c_str());
		DEBUG_MSG("\tDerror:%s",String(dError).c_str());
		DEBUG_MSG("\tAngPIDOut:%s",String(angPIDout).c_str());
	}

	// Debug Information
	Serial.printf("%s,",String(kAngle).c_str());
	Serial.printf("%s,",String(kRefAngle).c_str());
	Serial.printf("%s,",String(kRpm).c_str());
	// Serial.printf("%s,",String(Rpm_ControlOutput).c_str());
	// Serial.printf("%s,",String(Curr_target).c_str());
	Serial.printf("%s,",String(kCurrent).c_str());
	// Serial.printf("%s",String(Curr_ControlOutput).c_str());
	Serial.println("");

	return angPIDout;
}



/*************************** MOTOR OPERATIONS *****************************************/
// Enables the Motor Related Functionality
void cQuark::driveOn()			// Enables Motor Drive (Enable Interrupt)
{
	if(kQuarkState!=QExecOn)
	{
		kQuarkState=QExecOn;											// Set Motor Mode
		cQuark::coast();
		DEBUG_MSG("\nDriveOn:");
	}
}


// Disables the Motor Related Functionality
void cQuark::driveOff()
{
	cQuark::coast();
	kQuarkState=QExecOff;
	DEBUG_MSG("\nDriveOff");
}							


// E-Brakes the Motor
void cQuark::brake()
{
	if(kQuarkState==QExecOn)
	{
		kMotorMode=Mode_BRAKE;
		kMotorCmdDirection=Direction_STOP;
		analogWrite(MOT_1,0);
		analogWrite(MOT_2,0);
		digitalWrite(MOT_1,HIGH);
		digitalWrite(MOT_2,HIGH);
		digitalWrite(MOT_1,LOW);
		digitalWrite(MOT_2,LOW);
	}
	DEBUG_MSG("\nBrake");
}								// Shorts motor terminals


// Sets the motor in a de-energized state
void cQuark::coast()
{
	if(kQuarkState==QExecOn)
	{
		kMotorMode=Mode_COAST;
		kMotorCmdDirection=Direction_STOP;
		analogWrite(MOT_1,0);
		analogWrite(MOT_2,0);
		digitalWrite(MOT_1,LOW);
		digitalWrite(MOT_2,LOW);
	}	
	DEBUG_MSG("\nCoast");
}


// Brakes the motor and ensures it comes to a stop before setting it to coast mode
void cQuark::stop()
{
	if(kQuarkState==QExecOn)
	{
		kStopCmdIssued=1;
		cQuark::brake();
		IsumMax_ang = 0;
		IsumMax_rpm = 0;
	}
	DEBUG_MSG("\nStop Cmd Issued");
}


// Resets any errors present and sets Quark in an initialized state.
void cQuark::ResetError()
{	
	if(kQuarkState==QError)
		kQuarkState=QExecOn;
	DEBUG_MSG("\nResetError");
}


/********************** SET OPERATIONS ***************************************/
// Used to set the RPM [-100 to 100]
// Return 1: Successful, 0: Rpm Not Set
bool cQuark::setRpm(int Rpm=nil)
{
	if(Rpm!=nil && kQuarkState==QExecOn)
	{
		if(abs(Rpm)<=100)
		{
			cQuark::kMotorMode=Mode_RPM;
			cQuark::kRefRpm=Rpm;
		}
		DEBUG_MSG("\nRPM Set: ");DEBUG_NUM(kRefRpm);
		return 1;
	}
	else
	{
		DEBUG_MSG("\nRPM NOT Set!! State:%d",kQuarkState);
		return 0;
	}
}				// -90 rpm < RPM   < 90 rpm


// Used to set the angle of the motor [-160 to 160]
// Return 1: Successful, 0: Rpm Not Set
bool cQuark::setAngle(int Angle=nil)
{
	if(Angle!=nil && kQuarkState==QExecOn)
	{
		Serial.print("Current current: ");
		Serial.println(Quark.getCurrent());
		float currentDifference;
		int currentSlope = 0;
		/*if (Quark.getCurrent() < 90) {
			if(abs(Angle)<=160)
			{
				cQuark::kMotorMode=Mode_ANGLE;
				kRefAngle=Angle;
			}
			DEBUG_MSG("\nAngle Set: ");DEBUG_NUM(kRefAngle);
			return 1;
		} else {
			Serial.println("STOPPING");
			Quark.stop();
			delay(500);
			return 1;
		}*/
		if (cQuark::oldestCurrent >= cQuark::rollingSize) {
			cQuark::oldestCurrent = 0;
		}
		cQuark::rollingCurrents[cQuark::oldestCurrent++] = Quark.getCurrent();
		
		for(int i = 0; i < cQuark::rollingSize; i++) {
			currentSlope += cQuark::rollingCurrents[i];
		}
		currentSlope /= cQuark::rollingSize;
		Serial.print("CurrentSlope: ");
		Serial.println(currentSlope);
		if (currentSlope < 170) {
			if (abs(Angle) <= 160) {
				cQuark::kMotorMode = Mode_ANGLE;
				cQuark::kRefAngle = Angle;
			}
			return 1;
		} else {
			Serial.println("STOPPING");
			cQuark::setrunEStopLED(1);
			Quark.stop();
			delay(500);
			for(int i = 0; i < cQuark::rollingSize; i++) {
				cQuark::rollingCurrents[i] = 0;
			}

			return 1;
		}

		/*currentDifference = cQuark::rollingCurrents[7] - cQuark::rollingCurrents[0];
		Serial.print("Current Difference: ");
		Serial.println(currentDifference);
		Serial.print("Target Angle: ");
		Serial.println(Angle);
		if (currentDifference < 125.0) {
			if (abs(Angle) <= 160) {
				cQuark::kMotorMode = Mode_ANGLE;
				cQuark::kRefAngle;
			}
			return 1;
		} else {
			Serial.println("STOPPING");
			cQuark::setrunEStopLED(1);
			Quark.stop();
			delay(500);
			for(int i = 0; i < cQuark::rollingSize; i++) {
				cQuark::rollingCurrents[i] = 0;
			}
			return 1;
		}*/
	}

	else
	{
		DEBUG_MSG("\nAngle NOT Set!! State:%d",kQuarkState);
		return 0;
	}
}				// -160deg < Angle < 160deg

void cQuark::setrunEStopLED(int run) {
	cQuark::runEStopLED = run;
}

int cQuark::getrunEStopLED() {
	return cQuark::runEStopLED;
}


// Sets the PID PWM frequency. Not this changes overall PWM Frequency
void cQuark::setPwmFreq(unsigned int Freq=nil)
{
	if(Freq!=nil) 	cQuark::PwmFreq=Freq;
	analogWriteFreq(Freq);
	DEBUG_MSG("\nPWM Freq Set: ");DEBUG_NUM(cQuark::PwmFreq);
}


// Set Current Error Callback
void cQuark::setCurrentErrorCb(ERR_CUR_HANDLER cb)
{
	_err_cur_Cb=cb;
}


/***************************** GET OPERATIONS **********************************/
// To Read Operational State
int cQuark::getState()
{
	DEBUG_MSG("\ngetMode: ");DEBUG_NUM(kQuarkState);
	return kQuarkState;
}				


// To Read Motor Mode (Brake, Angle, Coast or RPM modes)
int cQuark::getMode()
{
	DEBUG_MSG("\ngetMode: ");DEBUG_NUM(kMotorMode);
	return kMotorMode;
}		


// Returns RPM [-100rpm to 100rpm]
float cQuark::getRpm()
{
	DEBUG_MSG("\ngetRpm: ");DEBUG_NUM(kRpm);
	return kRpm;
}


// Returns Angle of Motor [-160 to 160]
int cQuark::getAngle()
{
	// DEBUG_MSG("\ngetAngle: ");DEBUG_NUM(kAngle);
	return kAngle;
}


// Returns Current through motor
float cQuark::getCurrent()
{
	DEBUG_MSG("\ngetCurrent: ");DEBUG_NUM(kCurrent);
	return kCurrent;
}




/*
/********************** Support Hardware Access Functions *********************************/
/** Overload PinCommand Operators **/
/*extern "C" int __analogRead(uint8_t pin);
extern "C" int __analogWrite(uint8_t pin,int val);
extern "C" int __digitalRead(uint8_t pin);
extern "C" void __digitalWrite(uint8_t pin,uint8_t val);

int cQuark::analogRead(uint8_t pin)
{
	if(pin>=100 && pin<104)
	{
		digitalWrite(S2,((pin-100)&0b100?HIGH:LOW));
		digitalWrite(S1,((pin-100)&0b010?HIGH:LOW));
		digitalWrite(S0,((pin-100)&0b001?HIGH:LOW));
		delayMicroseconds(10);
		return analogRead(COM);
	}
	else 
		return digitalRead(pin)*1023;
} 


extern int digitalRead(uint8_t pin)
{
	if(pin>=100)
		return analogRead(pin)>512;
	else if(pin>=20)
		return __digitalRead(pin - 20);
	else
		return __digitalRead(pin);
} */


/*digitalWrite and analogWrite are removed on purpose.
analogWrite(used in motor control) uses digitalWrite within thereby preventing PWM on motor pins
Provide a User Tip to not use these pins.

extern void digitalWrite(uint8_t pin,uint8_t val)
{
	switch(pin)
	{
		case D0:
		case D1:
		case D2:
		case D3:
		case D4:
		case D5:
				DEBUG_MSG("\ndigitalWrite");DEBUG_NUM(pin);DEBUG_MSG(":");DEBUG_NUM(val);
				__digitalWrite(pin,val); 
				return; 
				break;
		default:return;
	}
}


extern void analogWrite(uint8_t pin,int val)
{
	switch(pin)
	{
		case D0:
		case D1:
		case D2:
		case D3:
		case D4:
		case D5:
				DEBUG_MSG("\nAnalogWrite");DEBUG_NUM(pin);DEBUG_MSG(":");DEBUG_NUM(val);
				__analogWrite(pin,val); 
				return; 
				break;
		default:return;
	}
}*/