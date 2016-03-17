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

#define NUMBER_VARIABLES	21
#define NUMBER_FUNCTIONS	25
#define CHAR_STRINGS	
#include <aREST.h>

/**< Low battery fault bit  */
#define FAULT_LOW_BAT	0x0001		
/**< Sensor fault bit     	*/
#define FAULT_SENSOR	0x0002   
/**< No signal fault bit  	*/  	
#define FAULT_NO_SIGNAL	0x0004     	
/**< Beaglebone fault bit   */
#define FAULT_BB_FAULT	0x0008   
/**< NVM fault bit      	*/  	
#define FAULT_NVM		0x0010     

#define REST_ID			"255"
#define REST_NAME		"ArduinoHackerBoat"

//#define ARDUINO_STATE_LEN	13
//#define BONE_STATE_LEN		19

/**
 * @brief An enum to store the current state of the boat.
 */
typedef enum arduinoMode {
	ARD_POWERUP     	= 0,  		/**< The boat enters this state at the end of initialization */
	ARD_ARMED			= 1,  		/**< In this state, the boat is ready to receive go commands over RF */
	ARD_SELFTEST 	  	= 2,  		/**< After powerup, the boat enters this state to determine whether it's fit to run */
	ARD_DISARMED 	  	= 3,  		/**< This is the default safe state. No external command can start the motor */
	ARD_ACTIVE   	  	= 4,  		/**< This is the normal steering state */
	ARD_LOWBATTERY   	= 5,  		/**< The battery voltage has fallen below that required to operate the motor */
	ARD_FAULT    		= 6,  		/**< The boat is faulted in some fashion */
	ARD_SELFRECOVERY 	= 7,   		/**< The Beaglebone has failed and/or is not transmitting, so time to self-recover*/
	ARD_ARMEDTEST		= 8,		/**< The Arduino is accepting specific pin read/write requests for hardware testing. */
	ARD_ACTIVERUDDER	= 9,		/**< The Arduino is accepting direct rudder commands */
	ARD_NONE			= 10		/**< Provides a null value for no command yet received */
} arduinoState;        

const uint8_t arduinoModeCount = 11;
const String arduinoModes[] = {
	"PowerUp", 
	"Armed", 
	"SelfTest", 
	"Disarmed", 
	"Active", 
	"LowBattery", 
	"Fault", 
	"SelfRecovery", 
	"ArmedTest", 
	"ActiveRudder", 
	"None"};
						
/**
 * @brief Beaglebone state
 */
typedef enum boatMode {
	BOAT_START			= 0,  		/**< Initial starting state         */
	BOAT_SELFTEST		= 1,  		/**< Initial self-test            */
	BOAT_DISARMED		= 2,  		/**< Disarmed wait state          */  
	BOAT_FAULT			= 3,		/**< Beaglebone faulted           */ 
	BOAT_ARMED			= 4,		/**< Beaglebone armed & ready to navigate */ 
	BOAT_MANUAL			= 5,		/**< Beaglebone manual steering       */ 
	BOAT_WAYPOINT		= 6,		/**< Beaglebone navigating by waypoints   */
	BOAT_NOSIGNAL		= 7,		/**< Beaglebone has lost shore signal    */
	BOAT_RETURN			= 8,		/**< Beaglebone is attempting to return to start point */
	BOAT_ARMEDTEST		= 9,		/**< Beaglebone accepts all commands that would be valid in any unsafe state */
	BOAT_UNKNOWN		= 10		/**< State of the Beaglebone is currently unknown	*/
} boneState;

const uint8_t boatModeCount = 11;
const String boatModes[] = {
	"Start", 
	"SelfTest", 
	"Disarmed", 
	"Fault",
	"Armed", 
	"Manual", 
	"WaypointNavigation",
	"LossOfSignal", 
	"ReturnToLaunch", 
	"ArmedTest",
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
	arduinoMode 	mode;					/**< The current mode of the arduino                    */
	arduinoMode		command;
	throttleState 	throttle;   			/**< The current throttle position                    */
	boatMode 		boat;					/**< The current mode of the BeagleBone                */
	sensors_vec_t 	orientation;			/**< The current accelerometer tilt and magnetic heading of the boat  */
	float 			headingTarget;			/**< The desired magnetic heading                     */  
	float 			internalVoltage;		/**< The battery voltage measured on the control PCB          */
	float 			batteryVoltage;			/**< The battery voltage measured at the battery            */
	float			motorVoltage;
	uint8_t			enbButton;				/**< State of the enable button. off = 0; on = 0xff           */
	uint8_t	 		stopButton;				/**< State of the emergency stop button. off = 0; on = 0xff       */
	long 			timeSinceLastPacket;	/**< Number of milliseconds since the last command packet received    */
	long 			timeOfLastPacket;		/**< Time the last packet arrived */
	long 			timeOfLastBoneHB;	
	long 			timeOfLastShoreHB;
	uint8_t			faultString;			/**< Fault string -- binary string to indicate source of faults */
	float 			rudder;
	uint16_t		rudderRaw;
	uint16_t		internalVoltageRaw;
	uint16_t		motorVoltageRaw;
	float			motorCurrent;
	uint16_t		motorCurrentRaw;
	float			Kp;
	float			Ki;
	float			Kd;
	float 			magX;
	float 			magY;
	float 			magZ;
	float 			accX;
	float 			accY;
	float 			accZ;
	float 			gyroX;
	float 			gyroY;
	float 			gyroZ;
	uint8_t 		horn;
	uint8_t			motorDirRly;
	uint8_t			motorWhtRly;
	uint8_t			motorYlwRly;
	uint8_t			motorRedRly;
	uint8_t			motorRedWhtRly;
	uint8_t			motorRedYlwRly;
	uint8_t			servoPower;
	long 			startStopTime;
	long			startStateTime;
	arduinoMode		originMode;
} boatVector;

///////////////////////////////////
// general function declarations //
///////////////////////////////////

void initIO 	(void);
void initREST 	(aREST * rest, boatVector * thisBoat);
void initBoat	(boatVector * thisBoat);
void input		(aREST * rest, boatVector * thisBoat);
void output		(boatVector * thisBoat);

////////////////////////////////
// REST function declarations //
////////////////////////////////

int writeBoatMode 		(String params);		
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
int	boatHeartBeat		(String params);
int	shoreHeartBeat		(String params);
int dumpCoreState		(String params);
int dumpOrientationState(String params);
int dumpInputState		(String params);
int dumpRawInputState	(String params);
int dumpOutputState		(String params);

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