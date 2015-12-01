#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_NeoPixel.h>
#include "hardwareConfig.h"
#include "sensorCalibration.h"

#ifndef HACKERBOATLIB_h
#define HACKERBOATLIB_h

#define NUMBER_VARIABLES	30
#define NUMBER_FUNCTIONS	20
#include <aREST.h>

#define FAULT_LOW_BAT	0x0001		/**< Low battery fault bit  */
#define FAULT_SENSOR	0x0002     	/**< Sensor fault bit     */
#define FAULT_NO_SIGNAL	0x0004     	/**< No signal fault bit  */
#define FAULT_BB_FAULT	0x0008     	/**< Beaglebone fault bit   */
#define FAULT_NVM		0x0010     	/**< NVM fault bit      */

#define REST_ID			"255"
#define REST_NAME		"ArduinoHackerBoat"

/**
 * @brief An enum to store the current state of the boat.
 */
typedef enum arduinoState {
	BOAT_POWERUP     	= 0,  		/**< The boat enters this state at the end of initialization */
	BOAT_ARMED			= 1,  		/**< In this state, the boat is ready to receive go commands over RF */
	BOAT_SELFTEST   	= 2,  		/**< After powerup, the boat enters this state to determine whether it's fit to run */
	BOAT_DISARMED   	= 3,  		/**< This is the default safe state. No external command can start the motor */
	BOAT_ACTIVE     	= 4,  		/**< This is the normal steering state */
	BOAT_LOWBATTERY   	= 5,  		/**< The battery voltage has fallen below that required to operate the motor */
	BOAT_FAULT    		= 6,  		/**< The boat is faulted in some fashion */
	BOAT_SELFRECOVERY 	= 7,   		/**< The Beaglebone has failed and/or is not transmitting, so time to self-recover*/
	BOAT_ARMEDTEST		= 8,		/**< The Arduino is accepting specific pin read/write requests for hardware testing. */
	BOAT_ACTIVERUDDER	= 9,		/**< The Arduino is accepting direct rudder commands */
	BOAT_NONE			= 10		/**< Provides a null value for no command yet received */
} arduinoState;        

const uint8_t arduinoStateCount = 11;
const String arduinoStates[] = {"PowerUp", "Armed", "SelfTest", 
								"Disarmed", "Active", "LowBattery", 
								"Fault", "SelfRecovery", "ArmedTest", 
								"ActiveRudder", "None"};
						
/**
 * @brief Beaglebone state
 */
typedef enum boneState {
	BONE_START			= 0,  		/**< Initial starting state         */
	BONE_SELFTEST		= 1,  		/**< Initial self-test            */
	BONE_DISARMED		= 2,  		/**< Disarmed wait state          */  
	BONE_FAULT			= 3,		/**< Beaglebone faulted           */ 
	BONE_ARMED			= 4,		/**< Beaglebone armed & ready to navigate */ 
	BONE_MANUAL			= 5,		/**< Beaglebone manual steering       */ 
	BONE_WAYPOINT		= 6,		/**< Beaglebone navigating by waypoints   */
	BONE_NOSIGNAL		= 7,		/**< Beaglebone has lost shore signal    */
	BONE_RETURN			= 8,		/**< Beaglebone is attempting to return to start point */
	BONE_ARMEDTEST		= 9,		/**< Beaglebone accepts all commands that would be valid in any unsafe state */
	BONE_UNKNOWN		= 10		/**< State of the Beaglebone is currently unknown	*/
} boneState;

const uint8_t boneStateCount = 11;
const String boneStates[] = {
	"Start", 
	"SelfTest", 
	"Disarmed", 
	"Fault",
	"Armed", 
	"Manual", 
	"WaypointNavigation",
	"LossOfSignal", 
	"ReturnToLaunch", 
	"ArmedTest"
	"Unknown"
};

/**
 * @brief An enum to store the current throttle state.
 */
typedef enum throttleState {
  FWD5    = 5,  					/**< Full forward speed. Red, white, and yellow all tied to V+, black tied to V- */
  FWD4    = 4,  					/**< Motor forward 4 */
  FWD3    = 3, 						/**< Motor forward 3 */
  FWD2    = 2, 						/**< Motor forward 2 */
  FWD1    = 1,  					/**< Motor forward 1 */
  STOP    = 0,  					/**< Motor off */
  REV1    = -1,  					/**< Motor reverse 1 */
  REV2    = -3,  					/**< Motor forward 2 */
  REV3    = -5	 					/**< Full reverse speed */
} throttleState;

/**
 * @brief List of variable names
 */
/*const String varList[] = {
	"state", 
	"boneState", 
	"command",
	"fault", 
	"throttle", 
	"headingTarget", 
	"headingDelta",
	"headingCurrent", 
	"rudder", 
	"rudderRaw", 
	"internalVoltage",
	"internalVoltageRaw", 
	"motorVoltage", 
	"motorVoltageRaw",
	"motorCurrent", 
	"motorCurrentRaw", 
	"Kp", 
	"Ki", 
	"Kd",
	"pitch", 
	"roll", 
	"accX", 
	"accY", 
	"accZ", 
	"magX",
	"magY", 
	"magZ", 
	"gyroX", 
	"gyroY", 
	"gyroZ", 
	"startButton",
	"stopButton", 
	"horn", 
	"motorDirRly", 
	"motorWhtRly",
	"motorYlwRly", 
	"motorRedRly", 
	"motorRedWhtRly",
	"motorRedYlwRly"
}*/

/**
 * @brief List of function names
 */
/*String funcList[] = {
	"writeBoneState", 
	"writeCommand", 
	"writeThrottle", 
	"writeHeadingTarget", 
	"writeHeadingDelta", 
	"writeRudder", 
	"writeKp", 
	"writeKi", 
	"writeKd", 
	"writeHorn", 
	"writeMotorDirRly", 
	"writeMotorWhtRly", 
	"writeMotorYlwRly", 
	"writeMotorRedRly", 
	"writeMotorRedWhtRly", 
	"writeMotorRedYlwRly", 
}*/

/**
 * @brief Structure to hold the boat's state data 
 */
typedef struct boatVector {
	arduinoState 	state;					/**< The current state of the boat                    */
	arduinoState	command;
	throttleState 	throttle;   			/**< The current throttle position                    */
	boneState 		bone;					/**< The current state of the BeagleBone                */
	sensors_vec_t 	orientation;			/**< The current accelerometer tilt and magnetic heading of the boat  */
	double 			headingTarget;			/**< The desired magnetic heading                     */  
	double 			internalVoltage;		/**< The battery voltage measured on the control PCB          */
	double 			batteryVoltage;			/**< The battery voltage measured at the battery            */
	double			motorVoltage;
	int 			enbButton;				/**< State of the enable button. off = 0; on = 0xff           */
	int		 		stopButton;				/**< State of the emergency stop button. off = 0; on = 0xff       */
	long 			timeSinceLastPacket;	/**< Number of milliseconds since the last command packet received    */
	long 			timeOfLastPacket;		/**< Time the last packet arrived */
	long 			timeOfLastBoneHB;	
	long 			timeOfLastShoreHB;
	String			stateString;
	String 			boneStateString;
	String			commandString;
	uint8_t			faultString;			/**< Fault string -- binary string to indicate source of faults */
	double 			headingCurrent;
	double 			rudder;
	int				rudderRaw;
	int				internalVoltageRaw;
	int				motorVoltageRaw;
	double			motorCurrent;
	int				motorCurrentRaw;
	double			Kp;
	double			Ki;
	double			Kd;
	double			pitch;
	double			roll;
	double 			magX;
	double 			magY;
	double 			magZ;
	double 			accX;
	double 			accY;
	double 			accZ;
	double 			gyroX;
	double 			gyroY;
	double 			gyroZ;
	int 			horn;
	int				motorDirRly;
	int				motorWhtRly;
	int				motorYlwRly;
	int				motorRedRly;
	int				motorRedWhtRly;
	int				motorRedYlwRly;
	int				servoPower;
} boatVector;

//////////////////////
// global variables //
//////////////////////

static aREST 		restInput 	= aREST();	/**< REST input object **/
static boatVector 	boat;					/**< Boat state vector            */

///////////////////////////////////
// general function declarations //
///////////////////////////////////

void initIO 	(void);
void initREST 	(aREST * rest, boatVector * thisBoat);
void initBoat	(boatVector * thisBoat);
void input		(boatVector * thisBoat);
void output		(boatVector * thisBoat);

////////////////////////////////
// REST function declarations //
////////////////////////////////

int writeBoneState 		(String params);		
int writeCommand 		(String params);
int writeThrottle 		(String params);
int writeHeadingTarget 	(String params);
int writeHeadingDelta 	(String params);
int writeRudder 		(String params);
int writeKp 			(String params);
int writeKi 			(String params);
int writeKd 			(String params);
int writeHorn 			(String params);
int writeMotorDirRly 	(String params);
int writeMotorWhtRly 	(String params);
int writeMotorYlwRly 	(String params);
int writeMotorRedRly 	(String params);
int writeMotorRedWhtRly (String params);
int writeMotorRedYlwRly (String params);
int writeServoPower		(String params);
int	boneHeartBeat		(String params);
int	shoreHeartBeat		(String params);

/////////////////////////////////
// state function declarations //
/////////////////////////////////

arduinoState executePowerUp			(boatVector * thisBoat, arduinoState lastState);
arduinoState executeSelfTest		(boatVector * thisBoat, arduinoState lastState);
arduinoState executeDisarmed		(boatVector * thisBoat, arduinoState lastState);
arduinoState executeFault			(boatVector * thisBoat, arduinoState lastState);
arduinoState executeArmed			(boatVector * thisBoat, arduinoState lastState);
arduinoState executeArmedTest		(boatVector * thisBoat, arduinoState lastState);
arduinoState executeActive			(boatVector * thisBoat, arduinoState lastState);
arduinoState executeActiveRudder	(boatVector * thisBoat, arduinoState lastState);
arduinoState executeLowBattery		(boatVector * thisBoat, arduinoState lastState);
arduinoState executeSelfRecovery	(boatVector * thisBoat, arduinoState lastState);

#endif