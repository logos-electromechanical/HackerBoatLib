#include "HackerBoatLib.h"

///////////////////////////
// file global variables //
///////////////////////////

extern aREST 			restInput;			/**< REST input object */
extern boatVector 		boat;				/**< boat state object */
Servo steeringServo;						/**< Servo object corresponding to the steering servo           */
float currentError;    						/**< Current heading error. This is a global so the PID can be as well  */
float targetError 	= 0;					/**< Desired heading error. This is a global so the PID can be as well  */
PID steeringPID((double *)&currentError, 	/**< PID loop object for driving the steering servo */
				(double *)&(boat.rudder), 
				(double *)&targetError, 
				0, 0, 0, 
				REVERSE); 
Adafruit_9DOF 					dof			= Adafruit_9DOF();						/**< IMU object               */
Adafruit_LSM303_Accel_Unified 	accel		= Adafruit_LSM303_Accel_Unified(30301); /**< Accelerometer object           */
Adafruit_LSM303_Mag_Unified   	mag			= Adafruit_LSM303_Mag_Unified(30302);   /**< Magnetometer object          */
Adafruit_L3GD20_Unified 		gyro		= Adafruit_L3GD20_Unified(20);			/**< Gyro object */
Adafruit_NeoPixel 				ardLights 	= Adafruit_NeoPixel(ardLightCount, arduinoLightsPin,  NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel 				boneLights 	= Adafruit_NeoPixel(boneLightCount, boneLightsPin,  NEO_GRB + NEO_KHZ800);
		
/////////////////////////////////
// local function declarations //
/////////////////////////////////

float 	getHeadingError (float heading, float headingSet);
void 	lightControl(arduinoMode state, boatMode bone);
void 	calibrateMag (sensors_event_t *magEvent);
void 	calibrateAccel (sensors_event_t *accelEvent);
void 	setThrottle (boatVector * thisBoat);

/////////////////////////////////////////
// general public function definitions //
/////////////////////////////////////////

/**
 * @brief Initialize Arduino I/O
 */
void initIO 	(void) {
	LogSerial.begin(LOGspeed);
	RESTSerial.begin(RESTspeed);
	pinMode(servoEnable, 		OUTPUT);
	pinMode(steeringPin, 		OUTPUT);
	pinMode(relaySpeedWht, 		OUTPUT);
	pinMode(relaySpeedWhtFB, 	INPUT);
	pinMode(relaySpeedYlw, 		OUTPUT);
	pinMode(relaySpeedYlwFB, 	INPUT);
	pinMode(relaySpeedRed, 		OUTPUT);
	pinMode(relaySpeedRedFB, 	INPUT);
	pinMode(relaySpeedRedWht, 	OUTPUT);
	pinMode(relaySpeedRedWhtFB, INPUT); 
	pinMode(relaySpeedRedYlw, 	OUTPUT);
	pinMode(relaySpeedRedYlwFB, INPUT);
	pinMode(hornPin, 			OUTPUT);
	pinMode(hornPinFB, 			INPUT);
	pinMode(arduinoLightsPin, 	OUTPUT);
	pinMode(boneLightsPin, 		OUTPUT);
	pinMode(enableButtonPin, 	INPUT);
	pinMode(stopButtonPin, 		INPUT);

	digitalWrite(servoEnable, 		LOW);
	digitalWrite(relayDir, 			LOW);
	digitalWrite(relaySpeedWht, 	LOW);
	digitalWrite(relaySpeedYlw, 	LOW);
	digitalWrite(relaySpeedRed, 	LOW);
	digitalWrite(relaySpeedRedWht, 	LOW);
	digitalWrite(relaySpeedRedYlw, 	LOW);
	digitalWrite(hornPin, 			LOW);
	//digitalWrite(relayAux1, LOW);

	RESTSerial.println("I live!");
	LogSerial.println("I live!");
	Wire.begin();

	if(!accel.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		LogSerial.println("Ooops, no LSM303 detected ... Check your wiring!");
	}
	if(!mag.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		LogSerial.println("Ooops, no LSM303 detected ... Check your wiring!");
	}
	if(!gyro.begin()) {
		/* There was a problem detecting the L3GD20 ... check your connections */
		LogSerial.println("Ooops, no L3GD20 detected ... Check your wiring!");
	}

	steeringPID.SetSampleTime(100);
	steeringPID.SetOutputLimits(pidMin, pidMax);
	steeringServo.attach(steeringPin);
	steeringPID.SetMode(AUTOMATIC);
	
	//Serial.println("Setup complete!");
}

/**
 * @brief Initialize Arduino REST interface
 */
void initREST 	(aREST * rest, boatVector * thisBoat) {
	
	// REST identity
	rest->set_id(REST_ID);
	rest->set_name(REST_NAME);
	
	// Variable assignments
	rest->variable("i_mode", 				(int *)&(thisBoat->mode));
	rest->variable("i_boatMode", 			(int *)&(thisBoat->boat));
	rest->variable("i_command",	 			(int *)&(thisBoat->command));
	rest->variable("i_fault", 				(int *)&(thisBoat->faultString));
	rest->variable("i_throttle", 			(int *)&(thisBoat->throttle));
	rest->variable("d_headingTarget", 		(float *)&(thisBoat->headingTarget));
	rest->variable("d_headingCurrent", 		(float *)&(thisBoat->orientation.heading));
	rest->variable("d_rudder", 				(float *)&(thisBoat->rudder));
	rest->variable("i_rudderRaw", 			(int *)&(thisBoat->rudderRaw));
	rest->variable("d_internalVoltage", 	&(thisBoat->internalVoltage));
	rest->variable("i_internalVoltageRaw", 	(int *)&(thisBoat->internalVoltageRaw));
	rest->variable("d_motorVoltage", 		&(thisBoat->motorVoltage));
	rest->variable("i_motorVoltageRaw", 	(int *)&(thisBoat->motorVoltageRaw));
	rest->variable("d_motorCurrent", 		&(thisBoat->motorCurrent));
	rest->variable("i_motorCurrentRaw", 	(int *)&(thisBoat->motorCurrentRaw));
	rest->variable("d_Kp", 					(float *)&(thisBoat->Kp));
	rest->variable("d_Ki", 					(float *)&(thisBoat->Ki));
	rest->variable("d_Kd", 					(float *)&(thisBoat->Kd));
	rest->variable("d_pitch", 				&(thisBoat->orientation.pitch));
	rest->variable("d_roll", 				&(thisBoat->orientation.roll));
	rest->variable("d_accX", 				&(thisBoat->accX));
	rest->variable("d_accY", 				&(thisBoat->accY));
	rest->variable("d_accZ", 				&(thisBoat->accZ));
	rest->variable("d_magX", 				&(thisBoat->magX));
	rest->variable("d_magY", 				&(thisBoat->magY));
	rest->variable("d_magZ", 				&(thisBoat->magZ));
	rest->variable("d_gyroX", 				&(thisBoat->gyroX));
	rest->variable("d_gyroY", 				&(thisBoat->gyroY));
	rest->variable("d_gyroZ", 				&(thisBoat->gyroZ));
	rest->variable("i_startButton", 		(int *)&(thisBoat->enbButton));
	rest->variable("i_stopButton", 			(int *)&(thisBoat->stopButton));
	rest->variable("i_horn", 				(int *)&(thisBoat->horn));
	rest->variable("i_motorDirRly", 		(int *)&(thisBoat->motorDirRly));
	rest->variable("i_motorWhtRly", 		(int *)&(thisBoat->motorWhtRly));
	rest->variable("i_motorYlwRly", 		(int *)&(thisBoat->motorYlwRly));
	rest->variable("i_motorRedRly", 		(int *)&(thisBoat->motorRedRly));
	rest->variable("i_motorRedWhtRly", 		(int *)&(thisBoat->motorRedWhtRly));
	rest->variable("i_motorRedYlwRly", 		(int *)&(thisBoat->motorRedYlwRly));
		
	// function assignments
	rest->function("f_writeBoatMode", 			writeBoatMode);
	rest->function("f_writeCommand", 			writeCommand);
	rest->function("f_writeThrottle", 			writeThrottle);
	rest->function("f_writeHeadingTarget", 		writeHeadingTarget);
	rest->function("f_writeHeadingDelta",		writeHeadingDelta);
	rest->function("f_writeRudder", 			writeRudder);
	rest->function("f_writeKp", 				writeKp);
	rest->function("f_writeKi", 				writeKi);
	rest->function("f_writeKd", 				writeKd);
	rest->function("f_writeHorn", 				writeHorn);
	rest->function("f_writeMotorDirRly", 		writeMotorDirRly);
	rest->function("f_writemotorWhtRly", 		writeMotorWhtRly);
	rest->function("f_writemotorYlwRly", 		writeMotorYlwRly);
	rest->function("f_writemotorRedRly", 		writeMotorRedRly);
	rest->function("f_writemotorRedWhtRly", 	writeMotorRedWhtRly);
	rest->function("f_writemotorRedYlwRly", 	writeMotorRedYlwRly);
	rest->function("f_boatHeartBeat", 			boatHeartBeat);
	rest->function("f_shoreHeartBeat", 			shoreHeartBeat);
	rest->function("f_dumpCoreState",			dumpCoreState);
	rest->function("f_dumpOrientationState",	dumpOrientationState);
	rest->function("f_dumpInputState",			dumpInputState);
	rest->function("f_dumpRawInputState",		dumpRawInputState);
	rest->function("f_dumpOutputState",			dumpOutputState);
	
	// Serial.println("REST setup complete");
}

/**
 * @brief Initialize Hackerboat systems
 */
void initBoat	(boatVector * thisBoat) {
	thisBoat->mode	 			= ARD_POWERUP;
	thisBoat->command			= ARD_NONE;
	thisBoat->throttle			= STOP;
	thisBoat->boat				= BOAT_UNKNOWN;
	thisBoat->faultString		= "";
	thisBoat->rudder 			= 0.0;
	thisBoat->Kp				= Kp_start;
	thisBoat->Ki				= Ki_start;
	thisBoat->Kd				= Kd_start;
	thisBoat->horn				= LOW;
	thisBoat->motorDirRly		= LOW;
	thisBoat->motorRedRly		= LOW;
	thisBoat->motorYlwRly		= LOW;
	thisBoat->motorWhtRly		= LOW;
	thisBoat->motorRedWhtRly	= LOW;
	thisBoat->motorRedYlwRly	= LOW;
	thisBoat->servoPower		= LOW;
	steeringPID.SetTunings(thisBoat->Kp, thisBoat->Ki, thisBoat->Kd);
	steeringPID.SetOutputLimits(pidMin, pidMax);
	steeringPID.SetMode(MANUAL);
  	ardLights.begin();
  	boneLights.begin();
	
	// Serial.println("Boat setup complete");
}

/**
 * @brief Gather all inputs to the Arduino component
 *
 * @param thisBoat The boat's state vector
 */
void input (aREST * rest, boatVector * thisBoat) {
	sensors_event_t accel_event;
	sensors_event_t mag_event;
	sensors_event_t gyro_event;
	uint8_t failCnt = 0;	
	
	// get & process the IMU data
	accel.getEvent(&accel_event);
	mag.getEvent(&mag_event);
	//gyro.getEvent(&gyro_event);
	calibrateMag(&mag_event);
	calibrateAccel(&accel_event);
	dof.accelGetOrientation(&accel_event, &(thisBoat->orientation));
	dof.magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event);
	dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &(thisBoat->orientation));
	thisBoat->magX				= mag_event.magnetic.x;
	thisBoat->magY				= mag_event.magnetic.y;
	thisBoat->magZ				= mag_event.magnetic.z;
	thisBoat->accX				= accel_event.acceleration.x;
	thisBoat->accY				= accel_event.acceleration.y;
	thisBoat->accZ				= accel_event.acceleration.z;
	thisBoat->gyroX				= gyro_event.gyro.x;
	thisBoat->gyroY				= gyro_event.gyro.y;
	thisBoat->gyroZ				= gyro_event.gyro.z;
	//Serial.println("Got I2C inputs!");
	
	// read digital inputs
	thisBoat->enbButton 		= digitalRead(enableButtonPin);
	thisBoat->stopButton 		= digitalRead(stopButtonPin);
	
	// read analog inputs
	thisBoat->internalVoltageRaw 	= analogRead(internalBatVolt);
	thisBoat->motorVoltageRaw		= analogRead(motorVolt);
	thisBoat->motorCurrentRaw		= analogRead(motorCurrent);
	thisBoat->internalVoltage		= thisBoat->internalVoltageRaw 	* internalBatVoltMult;
	thisBoat->motorVoltage			= thisBoat->motorVoltageRaw 	* motorVoltMult;
	thisBoat->motorCurrent			= (thisBoat->motorCurrentRaw 	+ motorCurrentOffset) * motorCurrentMult;
	thisBoat->batteryVoltage		= 0;
	//Serial.println("Got analog inputs!");

	// read REST
	while (RESTSerial.available()) {
		if (rest->handle(RESTSerial)) {
			Serial.println("Resetting time since last packet");
			thisBoat->timeSinceLastPacket = 0;
			thisBoat->timeOfLastPacket = millis();
		} else {
			thisBoat->timeSinceLastPacket = millis() - thisBoat->timeOfLastPacket;
		}
	}
	//Serial.println("Got REST!");
	
	// print to log
}

/**
 * @brief Write all outputs from the Arduino component
 *
 * @param thisBoat The boat's state vector
 */
void output (boatVector * thisBoat) {
	//Serial.println("Writing outputs...");
	setThrottle(thisBoat);
	lightControl(thisBoat->mode, thisBoat->boat);
	digitalWrite(servoEnable, thisBoat->servoPower);
	digitalWrite(relayDir, thisBoat->motorDirRly);
	digitalWrite(relaySpeedRed, thisBoat->motorRedRly);
	digitalWrite(relaySpeedWht, thisBoat->motorWhtRly);
	digitalWrite(relaySpeedYlw, thisBoat->motorYlwRly);
	digitalWrite(relaySpeedRedWht, thisBoat->motorRedWhtRly);
	digitalWrite(relaySpeedRedYlw, thisBoat->motorRedYlwRly);
	thisBoat->rudderRaw = map(((thisBoat->rudder)*10), (pidMin*10), (pidMax*10), servoMin, servoMax);
	steeringServo.writeMicroseconds(thisBoat->rudderRaw);	
}

////////////////////////////////
// local function definitions //
////////////////////////////////

/** 
 *  @brief generate heading error from current heading and desired heading
 *
 *  @param heading current magnetic heading
 *  @param headingSet desired magnetic heading
 *
 *  @return heading error, adjusted for the fact that it's circular.
 *
 */
float getHeadingError (float heading, float headingSet) {
  float result = headingSet - heading;
  
  if (result > 180) {result -= 360;}
  if (result < -180) {result += 360;}
  
  return result;
}

void lightControl(arduinoMode mode, boatMode boat) {
	static long lastChangeTime = millis();
	bool flashState = false;
	uint8_t pixelCounter;
	uint32_t color0;
	
	switch (mode) {
		case ARD_POWERUP:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = 0;
				} else {
					flashState = true;
					color0 = grn;
				}
			}
			break;
		case ARD_ARMED:
			color0 = blu;
			break;
		case ARD_SELFTEST:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = red;
				} else {
					flashState = true;
					color0 = wht;
				}
			}
			break;
		case ARD_DISARMED:
			color0 = amb;
			break;
		case ARD_ACTIVE:
			color0 = grn;
			break;
		case ARD_LOWBATTERY:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = amb;
				} else {
					flashState = true;
					color0 = red;
				}
			}
			break;
		case ARD_FAULT:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = 0;
				} else {
					flashState = true;
					color0 = red;
				}
			}
			break;
		case ARD_SELFRECOVERY:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = blu;
				} else {
					flashState = true;
					color0 = amb;
				}
			}
			break;
		case ARD_ARMEDTEST:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = 0;
				} else {
					flashState = true;
					color0 = amb;
				}
			}
			break;
		case ARD_ACTIVERUDDER:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = grn;
				} else {
					flashState = true;
					color0 = blu;
				}
			}
			break;
		case ARD_NONE:
		default:
			color0 = 0;
	}
	for (pixelCounter = 0; pixelCounter < ardLightCount; pixelCounter++) {
		ardLights.setPixelColor(pixelCounter, color0);
	}
	
	switch (boat) {
		case BOAT_START:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = 0;
				} else {
					flashState = true;
					color0 = grn;
				}
			}
			break;
		case BOAT_SELFTEST:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = 0;
				} else {
					flashState = true;
					color0 = amb;
				}
			}
			break;
		case BOAT_DISARMED:
			color0 = amb;
			break;
		case BOAT_FAULT:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = 0;
				} else {
					flashState = true;
					color0 = red;
				}
			}
			break;
		case BOAT_ARMED:
			color0 = blu;
			break;
		case BOAT_MANUAL:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = grn;
				} else {
					flashState = true;
					color0 = blu;
				}
			}
			break;
		case BOAT_WAYPOINT:
			color0 = grn;
			break;
		case BOAT_NOSIGNAL:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = red;
				} else {
					flashState = true;
					color0 = wht;
				}
			}
			break;
		case BOAT_RETURN:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = blu;
				} else {
					flashState = true;
					color0 = amb;
				}
			}
			break;
		case BOAT_ARMEDTEST:
			if ((millis() - lastChangeTime) > flashDelay) {
				lastChangeTime = millis();
				if (flashState) {
					flashState = false;
					color0 = 0;
				} else {
					flashState = true;
					color0 = amb;
				}
			}
			break;
		case BOAT_UNKNOWN:
		default:
			color0 = 0;
	}
	for (pixelCounter = 0; pixelCounter < boneLightCount; pixelCounter++) {
	    boneLights.setPixelColor(pixelCounter, color0);
	}
	
}

void calibrateMag (sensors_event_t *magEvent) {
  float *mag_X, *mag_Y, *mag_Z;
  mag_X = &(magEvent->magnetic.x);
  mag_Y = &(magEvent->magnetic.y);
  mag_Z = &(magEvent->magnetic.z);
  *mag_X += X_MAG_OFFSET;
  *mag_X *= X_MAG_GAIN;
  *mag_Y += Y_MAG_OFFSET;
  *mag_Y *= Y_MAG_GAIN;
  *mag_Z += Z_MAG_OFFSET;
  *mag_Z *= Z_MAG_GAIN;
}

void calibrateAccel (sensors_event_t *accelEvent) {
  float *accel_X, *accel_Y, *accel_Z;
  accel_X = &(accelEvent->acceleration.x);
  accel_Y = &(accelEvent->acceleration.y);
  accel_Z = &(accelEvent->acceleration.z);
  *accel_X += X_ACCEL_OFFSET;
  *accel_X *= X_ACCEL_GAIN;
  *accel_Y += Y_ACCEL_OFFSET;
  *accel_Y *= Y_ACCEL_GAIN;
  *accel_Z += Z_ACCEL_OFFSET;
  *accel_Z *= Z_ACCEL_GAIN;
}

/** 
 * @brief Given a throttle setting, set the throttle relay driver states
 *
 * @param thisBoat The boat's state vector
 */
void setThrottle (boatVector * thisBoat) {
	switch (thisBoat->throttle) {
		case (FWD5):
			thisBoat->motorDirRly = LOW;
			thisBoat->motorWhtRly = HIGH;
			thisBoat->motorRedRly = HIGH;
			thisBoat->motorYlwRly = HIGH;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = LOW;
			break;
		case (FWD4):
			thisBoat->motorDirRly = LOW;
			thisBoat->motorWhtRly = LOW;
			thisBoat->motorRedRly = LOW;
			thisBoat->motorYlwRly = HIGH;
			thisBoat->motorRedWhtRly = HIGH;
			thisBoat->motorRedYlwRly = LOW;
			break;
		case (FWD3):
			thisBoat->motorDirRly = LOW;
			thisBoat->motorWhtRly = HIGH;
			thisBoat->motorRedRly = LOW;
			thisBoat->motorYlwRly = HIGH;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = LOW;
			break;
		case (FWD2):
			thisBoat->motorDirRly = LOW;
			thisBoat->motorWhtRly = HIGH;
			thisBoat->motorRedRly = LOW;
			thisBoat->motorYlwRly = LOW;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = HIGH;
			break;
		case (FWD1):
			thisBoat->motorDirRly = LOW;
			thisBoat->motorWhtRly = HIGH;
			thisBoat->motorRedRly = LOW;
			thisBoat->motorYlwRly = LOW;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = LOW;
			break;
		case (STOP):
			thisBoat->motorDirRly = LOW;
			thisBoat->motorWhtRly = LOW;
			thisBoat->motorRedRly = LOW;
			thisBoat->motorYlwRly = LOW;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = LOW;
			break;
		case (REV1):
			thisBoat->motorDirRly = HIGH;
			thisBoat->motorWhtRly = HIGH;
			thisBoat->motorRedRly = LOW;
			thisBoat->motorYlwRly = LOW;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = LOW;
			break;
		case (REV2):
			thisBoat->motorDirRly = HIGH;
			thisBoat->motorWhtRly = HIGH;
			thisBoat->motorRedRly = LOW;
			thisBoat->motorYlwRly = HIGH;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = LOW;
			break;
		case (REV3):
			thisBoat->motorDirRly = HIGH;
			thisBoat->motorWhtRly = HIGH;
			thisBoat->motorRedRly = HIGH;
			thisBoat->motorYlwRly = HIGH;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = LOW;
			break;
		default:
			thisBoat->motorDirRly = LOW;
			thisBoat->motorWhtRly = LOW;
			thisBoat->motorRedRly = LOW;
			thisBoat->motorYlwRly = LOW;
			thisBoat->motorRedWhtRly = LOW;
			thisBoat->motorRedYlwRly = LOW;
			break;
  }
}

//////////////////////////////
// fault handling functions //
//////////////////////////////

bool insertFault (const String& fault, String& faultString) {
	if (!hasFault(fault, faultString)) {
		faultString += fault + ":";
	}
	return true;
}

bool hasFault (const String& fault, const String &faultString) {
	if (faultString.indexOf(fault) >= 0) return true;
	return false;
}

bool removeFault (const String& fault, String& faultString) {
	int start = faultString.indexOf(fault);
	if (start >= 0) {
		faultString.remove(start, (fault.length() + 1));
		return true;
	} else return false;
}

int faultCount (const String &faultString) {
	int cnt = 0;
	int ptr = 0;
	while (ptr >= 0) {
		ptr = faultString.indexOf(':', (ptr + 1));
		if (ptr > 0) cnt++;
	}
	return cnt;
}

///////////////////////////////
// REST function definitions //
///////////////////////////////

int writeBoatMode (String params) {
	Serial.print("Entering writeBoatMode function: "); Serial.println(params);
	for (uint8_t i = 0; i < boatModeCount; i++) {
		if(params.startsWith(boatModes[i])) {
			boat.boat = (boatMode)i;
			restInput.addToBuffer(F("{\"boat\":\""));
			restInput.addToBuffer(boatModes[boat.boat]);
			restInput.addToBuffer(F("\"}"));
			return 0;
		}
	}
	boat.boat = BOAT_UNKNOWN;
	restInput.addToBuffer(F("{\"boat\":\"None\"}"));
	return -1;
}

int writeCommand (String params) {
	for (uint8_t i = 0; i < arduinoModeCount; i++) {
		if(params.startsWith(arduinoModes[i])) {
			boat.command = (arduinoMode)i;
			restInput.addToBuffer(F("{\"command\": \""));
			restInput.addToBuffer(arduinoModes[boat.command]);
			restInput.addToBuffer(F("\"}"));
			return 0;
		}
	}
	boat.command = ARD_NONE;
	restInput.addToBuffer(F("{\"command\":\"None\"}"));
	return -1;
}

int writeThrottle (String params) {
	uint8_t throttleIn = params.toInt();
	if ((boat.mode == ARD_ACTIVE) || (boat.mode == ARD_ACTIVERUDDER)) {
		if (((throttleIn >= -1) && (throttleIn <= 5)) || 
			(throttleIn == -3) || (throttleIn == -5)) {
				boat.throttle = (throttleState)throttleIn;
		}
	}
	restInput.addToBuffer(F("{\"throttle\":"));
	restInput.addToBuffer(boat.throttle);
	restInput.addToBuffer(F("}"));
	return boat.throttle;
}

int writeHeadingTarget (String params) {
	double headingIn = params.toFloat();
	if (boat.mode == ARD_ACTIVE) {
		if ((headingIn <= 360) || (headingIn >= 0)) {
			boat.headingTarget = headingIn;
		}
	}
	restInput.addToBuffer(F("{\"headingTarget\":"));
	restInput.addToBuffer(boat.headingTarget);
	restInput.addToBuffer(F("}"));
	return (boat.headingTarget);
}

int writeHeadingDelta (String params) {
	double headingIn = params.toFloat();
	if (boat.mode == ARD_ACTIVE) {
		if ((headingIn <= 180) || (headingIn >= -180)) {
			boat.headingTarget += headingIn;
			if (boat.headingTarget < 0) boat.headingTarget += 360;
			if (boat.headingTarget > 360) boat.headingTarget -= 360;
		}
	}
	restInput.addToBuffer(F("{\"headingTarget\":"));
	restInput.addToBuffer(boat.headingTarget);
	restInput.addToBuffer(F("}"));
	return (boat.headingTarget);
}

int writeRudder (String params) {
	double rudderIn = params.toFloat();
	if ((boat.mode == ARD_ARMEDTEST) || 
		(boat.mode == ARD_ACTIVERUDDER) ||
		(boat.mode == ARD_ARMED)) {
			if ((rudderIn <= pidMax) || (rudderIn >= pidMin)) {
				boat.rudder = rudderIn;
			}
	}
	restInput.addToBuffer(F("{\"rudder\":"));
	restInput.addToBuffer(boat.rudder);
	restInput.addToBuffer(F("}"));
	return (boat.rudder * 10);
}

int writeKp (String params) {
	double Kin = params.toFloat();
	if ((Kin <= 10) || (Kin >= -10)) {
		boat.Kp = Kin;
		steeringPID.SetTunings(boat.Kp, boat.Ki, boat.Kd);
	}
	restInput.addToBuffer(F("{\"Kp\":"));
	restInput.addToBuffer(boat.Kp);
	restInput.addToBuffer(F("}"));
	return (boat.Kp * 1000);
}

int writeKi (String params) {
	double Kin = params.toFloat();
	if ((Kin <= 10) || (Kin >= -10)) {
		boat.Ki = Kin;
		steeringPID.SetTunings(boat.Kp, boat.Ki, boat.Kd);
	}
	restInput.addToBuffer(F("{\"Ki\":"));
	restInput.addToBuffer(boat.Ki);
	restInput.addToBuffer(F("}"));
	return (boat.Ki * 1000);
}

int writeKd (String params) {
	double Kin = params.toFloat();
	if ((Kin <= 10) || (Kin >= -10)) {
		boat.Kd = Kin;
		steeringPID.SetTunings(boat.Kp, boat.Ki, boat.Kd);
	}
	restInput.addToBuffer(F("{\"Kd\":"));
	restInput.addToBuffer(boat.Kd);
	restInput.addToBuffer(F("}"));
	return (boat.Kd * 1000);
}

int writeHorn (String params) {
	if (boat.mode == ARD_ARMEDTEST) {
		if (params.startsWith("ON")) {
			boat.horn = HIGH;
		} else if (params.startsWith("OFF")) {
			boat.horn = LOW;
		}
	}
	restInput.addToBuffer(F("{\"horn\":"));
	restInput.addToBuffer(boat.horn);
	restInput.addToBuffer(F("}"));
	return (boat.horn);
}

int writeMotorDirRly (String params) {
	if (boat.mode == ARD_ARMEDTEST) {
		if (params.startsWith("ON")) {
			boat.motorDirRly = HIGH;
		} else if (params.startsWith("OFF")) {
			boat.motorDirRly = LOW;
		}
	}
	restInput.addToBuffer(F("{\"motorDirRly\":"));
	restInput.addToBuffer(boat.motorDirRly);
	restInput.addToBuffer(F("}"));
	return (boat.motorDirRly);
}

int writeMotorWhtRly (String params) {
	if (boat.mode == ARD_ARMEDTEST) {
		if (params.startsWith("ON")) {
			boat.motorWhtRly = HIGH;
		} else if (params.startsWith("OFF")) {
			boat.motorWhtRly = LOW;
		}
	}
	restInput.addToBuffer(F("{\"motorWhtRly\":"));
	restInput.addToBuffer(boat.motorWhtRly);
	restInput.addToBuffer(F("}"));
	return (boat.motorWhtRly);
}

int writeMotorYlwRly (String params) {
	if (boat.mode == ARD_ARMEDTEST) {
		if (params.startsWith("ON")) {
			boat.motorYlwRly = HIGH;
		} else if (params.startsWith("OFF")) {
			boat.motorYlwRly = LOW;
		}
	}
	restInput.addToBuffer(F("{\"motorYlwRly\":"));
	restInput.addToBuffer(boat.motorYlwRly);
	restInput.addToBuffer(F("}"));
	return (boat.motorYlwRly);
}

int writeMotorRedRly (String params) {
	if (boat.mode == ARD_ARMEDTEST) {
		if (params.startsWith("ON")) {
			boat.motorRedRly = HIGH;
		} else if (params.startsWith("OFF")) {
			boat.motorRedRly = LOW;
		}
	}
	restInput.addToBuffer(F("{\"motorRedRly\":"));
	restInput.addToBuffer(boat.motorRedRly);
	restInput.addToBuffer(F("}"));
	return (boat.motorRedRly);
}

int writeMotorRedWhtRly (String params) {
	if (boat.mode == ARD_ARMEDTEST) {
		if (params.startsWith("ON")) {
			boat.motorRedWhtRly = HIGH;
		} else if (params.startsWith("OFF")) {
			boat.motorRedWhtRly = LOW;
		}
	}
	restInput.addToBuffer(F("{\"motorRedWhtRly\":"));
	restInput.addToBuffer(boat.motorRedWhtRly);
	restInput.addToBuffer(F("}"));
	return (boat.motorRedWhtRly);
}

int writeMotorRedYlwRly (String params) {
	if (boat.mode == ARD_ARMEDTEST) {
		if (params.startsWith("ON")) {
			boat.motorRedYlwRly = HIGH;
		} else if (params.startsWith("OFF")) {
			boat.motorRedYlwRly = LOW;
		}
	}
	restInput.addToBuffer(F("{\"motorRedYlwRly\":"));
	restInput.addToBuffer(boat.motorRedYlwRly);
	restInput.addToBuffer(F("}"));
	return (boat.motorRedYlwRly);
}

int writeServoPower (String params) {
	if (boat.mode == ARD_ARMEDTEST) {
		if (params.startsWith("ON")) {
			boat.servoPower = HIGH;
		} else if (params.startsWith("OFF")) {
			boat.servoPower = LOW;
		}
	}
	restInput.addToBuffer(F("{\"servoPower\":"));
	restInput.addToBuffer(boat.servoPower);
	restInput.addToBuffer(F("}"));
	return (boat.servoPower);
}

int	boatHeartBeat (String params) {
	boat.timeOfLastBoatHB = millis();
	restInput.addToBuffer(F("{\"heartbeat\":true}"));
	return 0;
}

int	shoreHeartBeat (String params) {
	boat.timeOfLastShoreHB = millis();
	restInput.addToBuffer(F("{\"heartbeat\":true}"));
	return 0;

}

int dumpCoreState (String params) {
	// valid JSON per http://jsonlint.com/
	restInput.addToBuffer(F("{\"mode\": \""));
	restInput.addToBuffer(arduinoModes[boat.mode]);
	restInput.addToBuffer(F("\",\"command\": \""));
	restInput.addToBuffer(arduinoModes[boat.command]);
	restInput.addToBuffer(F("\",\"throttle\":"));
	restInput.addToBuffer(boat.throttle);
	restInput.addToBuffer(F(",\n\"boat\": \""));
	restInput.addToBuffer(boatModes[boat.boat]);
	restInput.addToBuffer(F("\",\n\"headingTarget\":"));
	restInput.addToBuffer(boat.headingTarget);
	restInput.addToBuffer(F(",\"orientation\":{\"heading\":"));
	restInput.addToBuffer(boat.orientation.heading);
	restInput.addToBuffer(F(",\"pitch\":"));
	restInput.addToBuffer(boat.orientation.pitch);
	restInput.addToBuffer(F(",\"roll\":"));
	restInput.addToBuffer(boat.orientation.roll);
	restInput.addToBuffer(F("},\n\"faultString\":\""));
	restInput.addToBuffer(boat.faultString.c_str());
	restInput.addToBuffer(F("\",\n\"originMode\": \""));
	restInput.addToBuffer(arduinoModes[boat.originMode]);
	restInput.addToBuffer(F("\"}"));
	
	return 0;
}

int dumpOrientationState (String params) {
	// valid JSON per http://jsonlint.com/
	restInput.addToBuffer(F("{\"orientation\":{\"heading\":"));
	restInput.addToBuffer(boat.orientation.heading);
	restInput.addToBuffer(F(",\"pitch\":"));
	restInput.addToBuffer(boat.orientation.pitch);
	restInput.addToBuffer(F(",\"roll\":"));
	restInput.addToBuffer(boat.orientation.roll);
	restInput.addToBuffer(F("},\n\"magX\":"));
	restInput.addToBuffer(boat.magX);
	restInput.addToBuffer(F(",\n\"magY\":"));
	restInput.addToBuffer(boat.magY);
	restInput.addToBuffer(F(",\n\"magZ\":"));
	restInput.addToBuffer(boat.magZ);
	restInput.addToBuffer(F(",\n\"accX\":"));
	restInput.addToBuffer(boat.accX);
	restInput.addToBuffer(F(",\n\"accY\":"));
	restInput.addToBuffer(boat.accY);
	restInput.addToBuffer(F(",\n\"accZ\":"));
	restInput.addToBuffer(boat.accZ);
	restInput.addToBuffer(F(",\n\"gyroX\":"));
	restInput.addToBuffer(boat.gyroX);
	restInput.addToBuffer(F(",\n\"gyroY\":"));
	restInput.addToBuffer(boat.gyroY);
	restInput.addToBuffer(F(",\n\"gyroZ\":"));
	restInput.addToBuffer(boat.gyroZ);
	restInput.addToBuffer(F("}"));
	
	return 0;
}

int dumpInputState (String params) {
	// valid JSON per http://jsonlint.com/
	restInput.addToBuffer(F("{\"internalVoltage\":"));
	restInput.addToBuffer(boat.internalVoltage);
	restInput.addToBuffer(F(",\n\"batteryVoltage\":"));
	restInput.addToBuffer(boat.batteryVoltage);
	restInput.addToBuffer(F(",\n\"motorVoltage\":"));
	restInput.addToBuffer(boat.motorVoltage);
	restInput.addToBuffer(F(",\n\"enbButton\":"));
	restInput.addToBuffer(boat.enbButton);
	restInput.addToBuffer(F(",\n\"stopButton\":"));
	restInput.addToBuffer(boat.stopButton);
	restInput.addToBuffer(F(",\n\"rudder\":"));
	restInput.addToBuffer(boat.rudder);
	restInput.addToBuffer(F(",\n\"motorCurrent\":"));
	restInput.addToBuffer(boat.motorCurrent);
	restInput.addToBuffer(F("}"));
	
	return 0;
}

int dumpRawInputState (String params) {
	// valid JSON per http://jsonlint.com/
	restInput.addToBuffer(F("{\"rudderRaw\":"));
	restInput.addToBuffer(boat.rudderRaw);
	restInput.addToBuffer(F(",\n\"internalVoltageRaw\":"));
	restInput.addToBuffer(boat.internalVoltageRaw);
	restInput.addToBuffer(F(",\n\"motorVoltageRaw\":"));
	restInput.addToBuffer(boat.motorVoltageRaw);
	restInput.addToBuffer(F(",\n\"motorCurrentRaw\":"));
	restInput.addToBuffer(boat.motorCurrentRaw);
	restInput.addToBuffer(F("}"));
	
	return 0;
}

int dumpOutputState	(String params) {
	// valid JSON per http://jsonlint.com/
	restInput.addToBuffer(F("{\"horn\":"));
	restInput.addToBuffer(boat.horn);
	restInput.addToBuffer(F(",\n\"motorDirRly\":"));
	restInput.addToBuffer(boat.motorDirRly);
	restInput.addToBuffer(F(",\n\"motorWhtRly\":"));
	restInput.addToBuffer(boat.motorWhtRly);
	restInput.addToBuffer(F(",\n\"motorYlwRly\":"));
	restInput.addToBuffer(boat.motorYlwRly);
	restInput.addToBuffer(F(",\n\"motorRedRly\":"));
	restInput.addToBuffer(boat.motorRedRly);
	restInput.addToBuffer(F(",\n\"motorRedWhtRly\":"));
	restInput.addToBuffer(boat.motorRedWhtRly);
	restInput.addToBuffer(F(",\n\"motorRedYlwRly\":"));
	restInput.addToBuffer(boat.motorRedYlwRly);
	restInput.addToBuffer(F(",\n\"servoPower\":"));
	restInput.addToBuffer(boat.servoPower);
	restInput.addToBuffer(F("}"));
	
	return 0;
}
	

////////////////////////////////
// state function definitions //
////////////////////////////////

arduinoState executePowerUp (boatVector * thisBoat, arduinoState lastState) {
	// Power down all relays, power down the servo, and center the rudder
	thisBoat->motorDirRly 		= LOW;
	thisBoat->motorWhtRly 		= LOW;
	thisBoat->motorRedRly 		= LOW;
	thisBoat->motorYlwRly 		= LOW;
	thisBoat->motorRedWhtRly 	= LOW;
	thisBoat->motorRedYlwRly 	= LOW;
	thisBoat->rudder 			= 0;
	thisBoat->servoPower		= LOW;
	return ARD_SELFTEST;
}

arduinoState executeSelfTest (boatVector * thisBoat, arduinoState lastState) {
	static uint8_t faultCnt = 0;
	static uint8_t headingFaultCnt = 0;
	static uint8_t accelFaultCnt = 0;
	static uint8_t gyroFaultCnt = 0;
	static uint8_t signalFaultCnt = 0;

	LogSerial.println(F("**** Self-testing... ****"));

	// if we've just entered this state, reset all the counters
	if (lastState != ARD_SELFTEST) {
		faultCnt = 0; 
		thisBoat->headingTarget = thisBoat->orientation.heading;
		headingFaultCnt = 0;
		accelFaultCnt = 0;
		gyroFaultCnt = 0;
		signalFaultCnt = 0;
		thisBoat->startModeTime = millis();
	}
	//Serial.println("Running self test...");
	
	// Power down all relays, power up the servo, and center the rudder
	thisBoat->motorDirRly 		= LOW;
	thisBoat->motorWhtRly 		= LOW;
	thisBoat->motorRedRly 		= LOW;
	thisBoat->motorYlwRly 		= LOW;
	thisBoat->motorRedWhtRly 	= LOW;
	thisBoat->motorRedYlwRly 	= LOW;
	thisBoat->rudder 			= 0;
	thisBoat->servoPower		= HIGH;
	//Serial.println("Servo power on...");
	
	// if we're commanded into ArmedTest, go there
	if (thisBoat->command == ARD_ARMEDTEST) {
		thisBoat->command = ARD_NONE;
		return ARD_ARMEDTEST;
	}
	
	// check the battery
	if (thisBoat->internalVoltage < testVoltageLimit) {
		faultCnt++;
		insertFault("Low Battery", thisBoat->faultString);
	}
	
	// check the orientation
	if ((millis() - thisBoat->timeOfLastPacket) < sensorTestPeriod) {
		if ((thisBoat->orientation.roll > tiltDeviationLimit) || (thisBoat->orientation.pitch > tiltDeviationLimit)) {
			faultCnt++;
			LogSerial.print(F("Sensor outside of roll/pitch limits. Measure values roll: "));
			LogSerial.print(thisBoat->orientation.roll);
			LogSerial.print(F(" pitch: "));
			LogSerial.println(thisBoat->orientation.pitch);
			insertFault("Sensor Fault", thisBoat->faultString);
		}
		if (abs(getHeadingError(thisBoat->orientation.heading, thisBoat->headingTarget)) > compassDeviationLimit) {
			LogSerial.print(F("Compass outside of deviation limits. Compass heading: "));
			LogSerial.print(thisBoat->orientation.heading);
			LogSerial.print(F(" Reference: "));
			LogSerial.print(thisBoat->headingTarget);
			LogSerial.print(F(" Error: "));
			LogSerial.println(getHeadingError(thisBoat->orientation.heading, thisBoat->headingTarget));
			insertFault("Sensor Fault", thisBoat->faultString);
		}
	}
	//Serial.println(F("Orientation checked!"));
	
	// check for incoming signal
	if ((millis() - thisBoat->timeOfLastPacket) > signalTestPeriod) {
		insertFault("No Signal", thisBoat->faultString);
		LogSerial.print(F("Signal timeout. Current time: "));
		LogSerial.print(millis());
		LogSerial.print(F(" Last time: "));
		LogSerial.println(thisBoat->timeOfLastPacket);
	} else {
		removeFault("No Signal", thisBoat->faultString);
		LogSerial.print(F("Removing signal timeout. Current time: "));
		LogSerial.print(millis());
		LogSerial.print(F(" Last time: "));
		LogSerial.print(thisBoat->timeOfLastPacket);
		LogSerial.print(F(" Fault string: "));
		LogSerial.println(thisBoat->faultString);
	}
	//Serial.println(F("Incoming signal checked!"));
	
	// Check for fault from the Beaglebone
	if (BOAT_FAULT == thisBoat->boat) {
		insertFault("Bone Fault", thisBoat->faultString);
	}
	
	// Check for the end of the test
	if ((millis() - thisBoat->startModeTime) > startupTestPeriod) {
		if (faultCnt) {
			LogSerial.print(F("Got faults on startup. Fault string: \""));
			LogSerial.print(thisBoat->faultString);
			LogSerial.println("\"");
			return ARD_FAULT;
		} else {
			return ARD_DISARMED;
		}
	}
	//Serial.println(F("Returning from state function!"));
	
	return ARD_SELFTEST;
}

arduinoState executeDisarmed (boatVector * thisBoat, arduinoState lastState) {
	static long startEnbTime;

	LogSerial.println(F("**** Disarmed ****"));
	
	// Power down all relays & the servo, and center the rudder
	thisBoat->motorDirRly 		= LOW;
	thisBoat->motorWhtRly 		= LOW;
	thisBoat->motorRedRly 		= LOW;
	thisBoat->motorYlwRly 		= LOW;
	thisBoat->motorRedWhtRly 	= LOW;
	thisBoat->motorRedYlwRly 	= LOW;
	thisBoat->rudder 			= 0;
	thisBoat->servoPower		= LOW;
	thisBoat->throttle 			= STOP;
	
	// Track target heading to current heading
	thisBoat->headingTarget = thisBoat->orientation.heading;
	
	// check button state
	if (!(thisBoat->enbButton)) {
		startEnbTime = millis();
	}
   
	// determine next state
	if ((millis() - startEnbTime) > enbButtonTime) return ARD_ARMED;
	if (BOAT_FAULT == thisBoat->boat) {
		insertFault("Bone Fault", thisBoat->faultString);
		return ARD_FAULT;
	}
	if ((millis() - thisBoat->timeOfLastBoatHB) > disarmedPacketTimeout) {
		insertFault("No Signal", thisBoat->faultString);
		return ARD_FAULT;
	}
	return ARD_DISARMED;
}

arduinoState executeFault (boatVector * thisBoat, arduinoState lastState) {

	LogSerial.println(F("**** Fault ****"));
	
	// Power down all relays & the servo, and center the rudder
	thisBoat->motorDirRly 		= LOW;
	thisBoat->motorWhtRly 		= LOW;
	thisBoat->motorRedRly 		= LOW;
	thisBoat->motorYlwRly 		= LOW;
	thisBoat->motorRedWhtRly 	= LOW;
	thisBoat->motorRedYlwRly 	= LOW;
	thisBoat->rudder 			= 0;
	thisBoat->servoPower		= LOW;
	thisBoat->throttle 			= STOP;
	thisBoat->horn				= LOW;
	
	// Track target heading to current heading
	thisBoat->headingTarget = thisBoat->orientation.heading;
	
	LogSerial.print("Fault string: \"");
	LogSerial.print(thisBoat->faultString);
	LogSerial.println("\"");
	if (!faultCount(thisBoat->faultString)) return ARD_DISARMED;
	LogSerial.println("Faults still present");
	if (ARD_SELFTEST == thisBoat->command) return ARD_SELFTEST;

	return ARD_FAULT;
  
}

arduinoState executeArmed (boatVector * thisBoat, arduinoState lastState) {

	LogSerial.println(F("**** Armed ****"));
	
	// Power down all relays & the servo, and center the rudder
	thisBoat->motorDirRly 		= LOW;
	thisBoat->motorWhtRly 		= LOW;
	thisBoat->motorRedRly 		= LOW;
	thisBoat->motorYlwRly 		= LOW;
	thisBoat->motorRedWhtRly 	= LOW;
	thisBoat->motorRedYlwRly 	= LOW;
	thisBoat->rudder 			= 0;
	thisBoat->servoPower		= HIGH;
	thisBoat->throttle 			= STOP;
	
	// if we've just entered this state, reset all the counters and turn on the horn
	if (lastState != ARD_ARMED) {
		thisBoat->startModeTime 	= millis();
		thisBoat->originMode 		= lastState;
		thisBoat->horn 				= HIGH;
	}
	
	// check for the stop button
	if (!(thisBoat->stopButton)) {	
		// This resets the timer every time this state executes without the button pressed
		// As soon as the button is pressed, this is no longer reset and the timer runs.
		thisBoat->startStopTime = millis();
	}
	if ((millis() - thisBoat->startStopTime) > stopButtonTime) return ARD_DISARMED;
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		insertFault("Low Battery", thisBoat->faultString);
		thisBoat->horn = LOW;
		return ARD_LOWBATTERY;
	}
	
	// check for packet timeout
	if ((millis() - thisBoat->timeOfLastBoatHB) > armedPacketTimeout) {
		insertFault("No Signal", thisBoat->faultString);
		thisBoat->horn = LOW;
		return ARD_FAULT;
	}
	
	// Check to see if we came from a safe state and whether or not the horn timeout is over.
	// If we came from a safe state and the horn timeout is not over, sound the horn and reject
	// commands. 
	if (((millis() - thisBoat->startModeTime) < hornTimeout) && (ARD_DISARMED == thisBoat->originMode)) {
		thisBoat->horn = HIGH;
		return ARD_ARMED;
	} else {
		thisBoat->horn = LOW;
		if (ARD_DISARMED == thisBoat->command) {
			thisBoat->command = ARD_NONE;
			return ARD_DISARMED;
		}
		if (ARD_ACTIVE == thisBoat->command) {
			thisBoat->command = ARD_NONE;
			return ARD_ACTIVE;
		}
		if (ARD_ACTIVERUDDER == thisBoat->command) {
			thisBoat->command = ARD_NONE;
			return ARD_ACTIVERUDDER;
		}
	}
	
}

arduinoState executeArmedTest (boatVector * thisBoat, arduinoState lastState) {

	LogSerial.println(F("**** ArmedTest ****"));
	
	// reset timers & relays upon state entry
	if (lastState != ARD_ARMEDTEST) {
		thisBoat->startModeTime 	= millis();
		thisBoat->horn 				= HIGH;
		thisBoat->motorDirRly 		= LOW;
		thisBoat->motorWhtRly 		= LOW;
		thisBoat->motorRedRly 		= LOW;
		thisBoat->motorYlwRly 		= LOW;
		thisBoat->motorRedWhtRly 	= LOW;
		thisBoat->motorRedYlwRly 	= LOW;
		thisBoat->rudder 			= 0;
		thisBoat->servoPower		= LOW;
		thisBoat->throttle 			= STOP;
	}
	
	// check for the stop button
	if (!(thisBoat->stopButton)) {	
		// This resets the timer every time this state executes without the button pressed
		// As soon as the button is pressed, this is no longer reset and the timer runs.
		thisBoat->startStopTime = millis();
	}
	if ((millis() - thisBoat->startStopTime) > stopButtonTime) {		
		restInput.setEnable(AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
		return ARD_DISARMED;
	}
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		insertFault("Low Battery", thisBoat->faultString);
		thisBoat->horn = LOW;
		restInput.setEnable(AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
		return ARD_FAULT;
	}
	
	// check for packet timeout
	if ((millis() - thisBoat->timeOfLastShoreHB) > armedPacketTimeout) {
		insertFault("No Signal", thisBoat->faultString);
		thisBoat->horn = LOW;
		restInput.setEnable(AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
		return ARD_FAULT;
	}
	
	// Check to see if we came from a safe state and whether or not the horn timeout is over.
	// If we came from a safe state and the horn timeout is not over, sound the horn and reject
	// commands. 
	if ((millis() - thisBoat->startModeTime) < hornTimeout) {
		thisBoat->horn = HIGH;
		return ARD_ARMEDTEST;
	} else {
		thisBoat->horn = LOW;
		restInput.setEnable(AREST_ENB_DIGITAL | AREST_ENB_ANALOG | AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
		if (ARD_DISARMED == thisBoat->command) {
			thisBoat->command = ARD_NONE;
			restInput.setEnable(AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
			return ARD_DISARMED;
		}
		
	}
	
}

arduinoState executeActive (boatVector * thisBoat, arduinoState lastState) {

	LogSerial.println(F("**** Active ****"));
	
	if (lastState != ARD_ACTIVE) {
		thisBoat->horn 				= LOW;
		thisBoat->motorDirRly 		= LOW;
		thisBoat->motorWhtRly 		= LOW;
		thisBoat->motorRedRly 		= LOW;
		thisBoat->motorYlwRly 		= LOW;
		thisBoat->motorRedWhtRly 	= LOW;
		thisBoat->motorRedYlwRly 	= LOW;
		thisBoat->rudder 			= 0;
		thisBoat->servoPower		= HIGH;
		thisBoat->throttle 			= STOP;
	}
	
	// check for the stop button
	if (!(thisBoat->stopButton)) {	
		// This resets the timer every time this state executes without the button pressed
		// As soon as the button is pressed, this is no longer reset and the timer runs.
		thisBoat->startStopTime = millis();
	}
	if ((millis() - thisBoat->startStopTime) > stopButtonTime) {		
		steeringPID.SetMode(MANUAL);
		return ARD_DISARMED;
	}
	
	// Turn off the PID control if the throttle is not active
	if (STOP == thisBoat->throttle) {
		steeringPID.SetMode(MANUAL);
	} else {
		steeringPID.SetMode(AUTOMATIC);	// this is a no-op if it was already active
		steeringPID.Compute();
	}	
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		insertFault("Low Battery", thisBoat->faultString);
		steeringPID.SetMode(MANUAL);
		return ARD_LOWBATTERY;
	}
	
	// check for packet timeout
	if ((millis() - thisBoat->timeOfLastBoatHB) > armedPacketTimeout) {
		insertFault("No Signal", thisBoat->faultString);
		return ARD_SELFRECOVERY;
	}
	
	// check for commands
	if (ARD_ACTIVERUDDER == thisBoat->command) {
		thisBoat->command = ARD_NONE;
		steeringPID.SetMode(MANUAL);
		return ARD_ACTIVERUDDER;
	}
	if (ARD_DISARMED == thisBoat->command) {
		thisBoat->command = ARD_NONE;
		steeringPID.SetMode(MANUAL);
		return ARD_DISARMED;
	}
}

arduinoState executeActiveRudder (boatVector * thisBoat, arduinoState lastState)  {

	LogSerial.println(F("**** ActiveRudder ****"));
	
	if (lastState != ARD_ACTIVERUDDER) {
		thisBoat->horn 				= LOW;
		thisBoat->motorDirRly 		= LOW;
		thisBoat->motorWhtRly 		= LOW;
		thisBoat->motorRedRly 		= LOW;
		thisBoat->motorYlwRly 		= LOW;
		thisBoat->motorRedWhtRly 	= LOW;
		thisBoat->motorRedYlwRly 	= LOW;
		thisBoat->rudder 			= 0;
		thisBoat->servoPower		= HIGH;
	}
	
	// check for the stop button
	if (!(thisBoat->stopButton)) {	
		// This resets the timer every time this state executes without the button pressed
		// As soon as the button is pressed, this is no longer reset and the timer runs.
		thisBoat->startStopTime = millis();
	}
	if ((millis() - thisBoat->startStopTime) > stopButtonTime) {	
		return ARD_DISARMED;
	}
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		insertFault("Low Battery", thisBoat->faultString);
		return ARD_LOWBATTERY;
	}
	
	// check for packet timeout
	if ((millis() - thisBoat->timeOfLastShoreHB) > armedPacketTimeout) {
		insertFault("No Signal", thisBoat->faultString);
		return ARD_SELFRECOVERY;
	}
	
	// check for commands
	if (ARD_ACTIVE == thisBoat->command) {
		thisBoat->command = ARD_NONE;
		return ARD_ACTIVE;
	}
	if (ARD_DISARMED == thisBoat->command) {
		thisBoat->command = ARD_NONE;
		return ARD_DISARMED;
	}
}

arduinoState executeLowBattery (boatVector * thisBoat, arduinoState lastState) {

	LogSerial.println(F("**** Low Battery ****"));
	
	// Power down all relays & the servo, and center the rudder
	thisBoat->motorDirRly 		= LOW;
	thisBoat->motorWhtRly 		= LOW;
	thisBoat->motorRedRly 		= LOW;
	thisBoat->motorYlwRly 		= LOW;
	thisBoat->motorRedWhtRly 	= LOW;
	thisBoat->motorRedYlwRly 	= LOW;
	thisBoat->rudder 			= 0;
	thisBoat->servoPower		= LOW;
	thisBoat->throttle 			= STOP;
	
	// figure out what state we will return to 
	if (ARD_LOWBATTERY != lastState) {
		if ((ARD_ARMED == lastState) || 
			(ARD_ACTIVE == lastState) ||
			(ARD_ACTIVERUDDER == lastState) ||
			(ARD_SELFRECOVERY == lastState)) {
				thisBoat->originMode = lastState;
		} else thisBoat->originMode = ARD_DISARMED;
	}
	
	// check for the stop button
	if (!(thisBoat->stopButton)) {	
		// This resets the timer every time this state executes without the button pressed
		// As soon as the button is pressed, this is no longer reset and the timer runs.
		thisBoat->startStopTime = millis();
	}
	if ((millis() - thisBoat->startStopTime) > stopButtonTime) {	
		return ARD_DISARMED;
	}
	
	// check for battery voltage
	if (thisBoat->internalVoltage > recoverVoltageLimit) {
		removeFault("Low Battery", thisBoat->faultString);
		return thisBoat->originMode;
	}

	return ARD_LOWBATTERY;
}

arduinoState executeSelfRecovery (boatVector * thisBoat, arduinoState lastState) {
	
	LogSerial.println(F("**** SelfRecovery ****"));
	
	if (lastState != ARD_SELFRECOVERY) {
		thisBoat->horn 				= LOW;
		thisBoat->motorDirRly 		= LOW;
		thisBoat->motorWhtRly 		= LOW;
		thisBoat->motorRedRly 		= LOW;
		thisBoat->motorYlwRly 		= LOW;
		thisBoat->motorRedWhtRly 	= LOW;
		thisBoat->motorRedYlwRly 	= LOW;
		thisBoat->rudder 			= 0;
		thisBoat->servoPower		= HIGH;
		thisBoat->throttle 			= FWD5;
		steeringPID.SetMode(AUTOMATIC);
	}
	
	// check for the stop button
	if (!(thisBoat->stopButton)) {	
		// This resets the timer every time this state executes without the button pressed
		// As soon as the button is pressed, this is no longer reset and the timer runs.
		thisBoat->startStopTime = millis();
	}
	if ((millis() - thisBoat->startStopTime) > stopButtonTime) {		
		steeringPID.SetMode(MANUAL);
		return ARD_DISARMED;
	}
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		insertFault("Low Battery", thisBoat->faultString);
		steeringPID.SetMode(MANUAL);
		return ARD_LOWBATTERY;
	}
	
	// figure out what state we will return to 
	if (ARD_SELFRECOVERY != lastState) {
		if ((ARD_ARMED == lastState) || 
			(ARD_ACTIVE == lastState) ||
			(ARD_ACTIVERUDDER == lastState)) {
				thisBoat->originMode = lastState;
		} else thisBoat->originMode = ARD_DISARMED;
	}
	
	// check for heartbeat...
	if ((millis() - thisBoat->timeOfLastBoatHB) < armedPacketTimeout) {
		steeringPID.SetMode(MANUAL);
		return thisBoat->originMode;
	}
	
	return ARD_SELFRECOVERY;
}

