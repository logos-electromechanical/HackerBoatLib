#include "HackerBoatLib.h"

//////////////////////
// global variables //
//////////////////////

Servo steeringServo;			/**< Servo object corresponding to the steering servo           */
double currentError;    		/**< Current heading error. This is a global so the PID can be as well  */
double targetError 	= 0;		/**< Desired heading error. This is a global so the PID can be as well  */
double steeringCmd;				/**< Steering command to be written out to the servo.           */
PID steeringPID(&currentError, 	/**< PID loop object for driving the steering servo */
				&steeringCmd, 
				&targetError, 
				Kp, Ki, Kd, 
				REVERSE); 
Adafruit_9DOF dof 	= Adafruit_9DOF();	/**< IMU object               */
Adafruit_LSM303_Accel_Unified accel =   Adafruit_LSM303_Accel_Unified(30301); /**< Accelerometer object           */
Adafruit_LSM303_Mag_Unified   mag   =   Adafruit_LSM303_Mag_Unified(30302);   /**< Magnetometer object          */

/////////////////////////////////
// local function declarations //
/////////////////////////////////

double 	getHeadingError (double heading, double headingSet);
uint8_t getButtons (void);
void 	lightControl(boatState state, boneState bone);
void 	calibrateMag (sensors_event_t *magEvent);
void 	calibrateAccel (sensors_event_t *accelEvent);

//////////////////////////////
// general public functions //
//////////////////////////////

void initIO 	(void) {
	LogSerial.begin(115200);
	RestSerial.begin(115200);
	pinMode(servoEnable, OUTPUT);
	pinMode(steeringPin, OUTPUT);
	pinMode(relaySpeedWht, OUTPUT);
	pinMode(relaySpeedWhtFB, INPUT);
	pinMode(relaySpeedYlw, OUTPUT);
	pinMode(relaySpeedYlwFB, INPUT);
	pinMode(relaySpeedRed, OUTPUT);
	pinMode(relaySpeedRedFB, INPUT);
	pinMode(relaySpeedRedWht, OUTPUT);
	pinMode(relaySpeedRedWhtFB, INPUT); 
	pinMode(relaySpeedRedYlw, OUTPUT);
	pinMode(relaySpeedRedYlwFB, INPUT);
	pinMode(horn, OUTPUT);
	pinMode(hornFB, INPUT);
	pinMode(arduinoLightsPin, OUTPUT);
	pinMode(boneLightsPin, OUTPUT);
	pinMode(enableButton, INPUT);
	pinMode(stopButton, INPUT);

	digitalWrite(servoEnable, LOW);
	digitalWrite(relayDir, LOW);
	digitalWrite(relaySpeedWht, LOW);
	digitalWrite(relaySpeedYlw, LOW);
	digitalWrite(relaySpeedRed, LOW);
	digitalWrite(relaySpeedRedWht, LOW);
	digitalWrite(relaySpeedRedYlw, LOW);
	digitalWrite(horn, LOW);
	//digitalWrite(relayAux1, LOW);

	Serial.println("I live!");
	Wire.begin();

	if(!accel.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		LogSerial.println("Ooops, no LSM303 detected ... Check your wiring!");
	}
	if(!mag.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		LogSerial.println("Ooops, no LSM303 detected ... Check your wiring!");
	}

	steeringPID.SetSampleTime(100);
	steeringPID.SetOutputLimits(pidMin, pidMax);
	steeringServo.attach(steeringPin);
	steeringPID.SetMode(AUTOMATIC);
}

void initREST 	(aREST * rest, restStruct * vars) {
	rest->set_id(REST_ID);
	rest->set_name(REST_NAME);
	
	// Variable assignments
	rest->variable("state", 				&(vars->_state));
	rest->variable("boneState", 			&(vars->_boneState));
	rest->variable("command", 				&(vars->_command));
	rest->variable("fault", 				&(vars->_fault));
	rest->variable("throttle", 				&(vars->_throttle));
	rest->variable("headingTarget", 		&(vars->_headingTarget));
	rest->variable("headingDelta", 			&(vars->_headingDelta));
	rest->variable("headingCurrent", 		&(vars->_headingCurrent));
	rest->variable("rudder", 				&(vars->_rudder));
	rest->variable("rudderRaw", 			&(vars->_rudderRaw));
	rest->variable("internalVoltage", 		&(vars->_internalVoltage));
	rest->variable("internalVoltageRaw", 	&(vars->_internalVoltageRaw));
	rest->variable("motorVoltage", 			&(vars->_motorVoltage));
	rest->variable("motorVoltageRaw", 		&(vars->_motorVoltageRaw));
	rest->variable("motorCurrent", 			&(vars->_motorCurrent));
	rest->variable("motorCurrentRaw", 		&(vars->_motorCurrentRaw));
	rest->variable("Kp", 					&(vars->_Kp));
	rest->variable("Ki", 					&(vars->_Ki));
	rest->variable("Kd", 					&(vars->_Kd));
	rest->variable("pitch", 				&(vars->_pitch));
	rest->variable("roll", 					&(vars->_roll));
	rest->variable("accX", 					&(vars->_accX));
	rest->variable("accY", 					&(vars->_accY));
	rest->variable("accZ", 					&(vars->_accZ));
	rest->variable("magX", 					&(vars->_magX));
	rest->variable("magY", 					&(vars->_magY));
	rest->variable("magZ", 					&(vars->_magZ));
	rest->variable("gyroX", 				&(vars->_gyroX));
	rest->variable("gyroY", 				&(vars->_gyroY));
	rest->variable("gyroZ", 				&(vars->_gyroZ));
	rest->variable("startButton", 			&(vars->_startButton));
	rest->variable("stopButton", 			&(vars->_stopButton));
	rest->variable("horn", 					&(vars->_horn));
	rest->variable("motorDirRly", 			&(vars->_motorDirRly));
	rest->variable("motorWhtRly", 			&(vars->_motorWhtRly));
	rest->variable("motorYlwRly", 			&(vars->_motorYlwRly));
	rest->variable("motorRedRly", 			&(vars->_motorRedRly));
	rest->variable("motorRedWhtRly", 		&(vars->_motorRedWhtRly));
	rest->variable("motorRedYlwRly", 		&(vars->_motorRedYlwRly));
		
	// function assignments
	rest->function("writeBoneState", 		writeBoneState);
	rest->function("writeCommand", 			writeCommand);
	rest->function("writeThrottle", 		writeThrottle);
	rest->function("writeHeadingTarget", 	writeHeadingTarget);
	rest->function("writeHeadingDelta", 	writeHeadingDelta);
	rest->function("writeRudder", 			writeRudder);
	rest->function("writeKp", 				writeKp);
	rest->function("writeKi", 				writeKi);
	rest->function("writeKd", 				writeKd);
	rest->function("writeHorn", 			writeHorn);
	rest->function("writeMotorDirRly", 		writeMotorDirRly);
	rest->function("writemotorWhtRly", writemotorWhtRly);
	rest->function("writemotorYlwRly", writemotorYlwRly);
	rest->function("writemotorRedRly", writemotorRedRly);
	rest->function("writemotorRedWhtRly", writemotorRedWhtRly);
	rest->function("writemotorRedYlwRly", writemotorRedYlwRly);

}

void initBoat	(boatVector * thisBoat);
void input		(boatVector * thisBoat, restStruct * vars);
void output		(boatVector * thisBoat, restStruct * vars);