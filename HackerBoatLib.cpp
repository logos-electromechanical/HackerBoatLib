#include "HackerBoatLib.h"

///////////////////////////
// file global variables //
///////////////////////////

Servo steeringServo;			/**< Servo object corresponding to the steering servo           */
double currentError;    		/**< Current heading error. This is a global so the PID can be as well  */
double targetError 	= 0;		/**< Desired heading error. This is a global so the PID can be as well  */
double steeringCmd;				/**< Steering command to be written out to the servo.           */
PID steeringPID(&currentError, 	/**< PID loop object for driving the steering servo */
				&(boat.rudder), 
				&targetError, 
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

double 	getHeadingError (double heading, double headingSet);
void 	lightControl(arduinoState state, boneState bone);
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
	LogSerial.begin(115200);
	RESTSerial.begin(115200);
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
	if(!gyro.begin()) {
		/* There was a problem detecting the L3GD20 ... check your connections */
		LogSerial.println("Ooops, no L3GD20 detected ... Check your wiring!");
	}

	steeringPID.SetSampleTime(100);
	steeringPID.SetOutputLimits(pidMin, pidMax);
	steeringServo.attach(steeringPin);
	steeringPID.SetMode(AUTOMATIC);
}

/**
 * @brief Initialize Arduino REST interface
 */
void initREST 	(aREST * rest, boatVector * thisBoat) {
	
	// REST identity
	rest->set_id(REST_ID);
	rest->set_name(REST_NAME);
	
	// Variable assignments
	rest->variable("state", 				(int *)&(thisBoat->state));
	rest->variable("boneState", 			(int *)&(thisBoat->bone));
	rest->variable("command", 				(int *)&(thisBoat->command));
	rest->variable("fault", 				(int *)&(thisBoat->faultString));
	rest->variable("throttle", 				(int *)&(thisBoat->throttle));
	rest->variable("headingTarget", 		&(thisBoat->headingTarget));
	rest->variable("headingCurrent", 		&(thisBoat->headingCurrent));
	rest->variable("rudder", 				&(thisBoat->rudder));
	rest->variable("rudderRaw", 			&(thisBoat->rudderRaw));
	rest->variable("internalVoltage", 		&(thisBoat->internalVoltage));
	rest->variable("internalVoltageRaw", 	&(thisBoat->internalVoltageRaw));
	rest->variable("motorVoltage", 			&(thisBoat->motorVoltage));
	rest->variable("motorVoltageRaw", 		&(thisBoat->motorVoltageRaw));
	rest->variable("motorCurrent", 			&(thisBoat->motorCurrent));
	rest->variable("motorCurrentRaw", 		&(thisBoat->motorCurrentRaw));
	rest->variable("Kp", 					&(thisBoat->Kp));
	rest->variable("Ki", 					&(thisBoat->Ki));
	rest->variable("Kd", 					&(thisBoat->Kd));
	rest->variable("pitch", 				&(thisBoat->pitch));
	rest->variable("roll", 					&(thisBoat->roll));
	rest->variable("accX", 					&(thisBoat->accX));
	rest->variable("accY", 					&(thisBoat->accY));
	rest->variable("accZ", 					&(thisBoat->accZ));
	rest->variable("magX", 					&(thisBoat->magX));
	rest->variable("magY", 					&(thisBoat->magY));
	rest->variable("magZ", 					&(thisBoat->magZ));
	rest->variable("gyroX", 				&(thisBoat->gyroX));
	rest->variable("gyroY", 				&(thisBoat->gyroY));
	rest->variable("gyroZ", 				&(thisBoat->gyroZ));
	rest->variable("startButton", 			&(thisBoat->enbButton));
	rest->variable("stopButton", 			&(thisBoat->stopButton));
	rest->variable("horn", 					&(thisBoat->horn));
	rest->variable("motorDirRly", 			&(thisBoat->motorDirRly));
	rest->variable("motorWhtRly", 			&(thisBoat->motorWhtRly));
	rest->variable("motorYlwRly", 			&(thisBoat->motorYlwRly));
	rest->variable("motorRedRly", 			&(thisBoat->motorRedRly));
	rest->variable("motorRedWhtRly", 		&(thisBoat->motorRedWhtRly));
	rest->variable("motorRedYlwRly", 		&(thisBoat->motorRedYlwRly));
		
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
	rest->function("writemotorWhtRly", 		writeMotorWhtRly);
	rest->function("writemotorYlwRly", 		writeMotorYlwRly);
	rest->function("writemotorRedRly", 		writeMotorRedRly);
	rest->function("writemotorRedWhtRly", 	writeMotorRedWhtRly);
	rest->function("writemotorRedYlwRly", 	writeMotorRedYlwRly);
	rest->function("boneHeartBeat", 		boneHeartBeat);
	rest->function("shoreHeartBeat", 		boneHeartBeat);
}

/**
 * @brief Initialize Hackerboat systems
 */
void initBoat	(boatVector * thisBoat) {
	thisBoat->state 			= BOAT_POWERUP;
	thisBoat->command			= BOAT_NONE;
	thisBoat->throttle			= STOP;
	thisBoat->bone				= BONE_UNKNOWN;
	thisBoat->faultString		= LOW;
	thisBoat->rudder 			= LOW;
	thisBoat->stateString 		= arduinoStates[thisBoat->state];
	thisBoat->commandString		= arduinoStates[thisBoat->command];
	thisBoat->boneStateString	= boneStates[thisBoat->bone];
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
}

/**
 * @brief Gather all inputs to the Arduino component
 *
 * @param thisBoat The boat's state vector
 */
void input (boatVector * thisBoat) {
	sensors_event_t accel_event;
	sensors_event_t mag_event;
	sensors_event_t gyro_event;
	uint8_t failCnt = 0;	
	
	// get & process the IMU data
	accel.getEvent(&accel_event);
	mag.getEvent(&mag_event);
	gyro.getEvent(&gyro_event);
	calibrateMag(&mag_event);
	calibrateAccel(&accel_event);
	dof.accelGetOrientation(&accel_event, &(thisBoat->orientation));
	dof.magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event);
	dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &(thisBoat->orientation));
	thisBoat->headingCurrent 	= thisBoat->orientation.heading;
	thisBoat->pitch				= thisBoat->orientation.pitch;
	thisBoat->roll				= thisBoat->orientation.roll;
	thisBoat->magX				= mag_event.magnetic.x;
	thisBoat->magY				= mag_event.magnetic.y;
	thisBoat->magZ				= mag_event.magnetic.z;
	thisBoat->accX				= accel_event.acceleration.x;
	thisBoat->accY				= accel_event.acceleration.y;
	thisBoat->accZ				= accel_event.acceleration.z;
	thisBoat->gyroX				= gyro_event.gyro.x;
	thisBoat->gyroY				= gyro_event.gyro.y;
	thisBoat->gyroZ				= gyro_event.gyro.z;
	
	// read digital inputs
	thisBoat->enbButton 		= digitalRead(enableButton);
	thisBoat->stopButton 		= digitalRead(stopButton);
	
	// read analog inputs
	thisBoat->internalVoltageRaw 	= analogRead(internalBatVolt);
	thisBoat->motorVoltageRaw		= analogRead(motorVolt);
	thisBoat->motorCurrentRaw		= analogRead(motorCurrent);
	thisBoat->internalVoltage		= thisBoat->internalVoltageRaw 	* internalBatVoltMult;
	thisBoat->motorVoltage			= thisBoat->motorVoltageRaw 	* motorVoltMult;
	thisBoat->motorCurrent			= (thisBoat->motorCurrentRaw 	+ motorCurrentOffset) * motorCurrentMult;
	thisBoat->batteryVoltage		= 0;

	// read REST
	if (restInput.handle(RESTSerial)) {
		thisBoat->timeSinceLastPacket = 0;
		thisBoat->timeOfLastPacket = millis();
	} else {
		thisBoat->timeSinceLastPacket = millis() - thisBoat->timeOfLastPacket;
	}
	
	// print to log
}

/**
 * @brief Write all outputs from the Arduino component
 *
 * @param thisBoat The boat's state vector
 */
void output (boatVector * thisBoat) {
	setThrottle(thisBoat);
	lightControl(thisBoat->state, thisBoat->bone);
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
double getHeadingError (double heading, double headingSet) {
  double result = headingSet - heading;
  
  if (result > 180) {result -= 360;}
  if (result < -180) {result += 360;}
  
  return result;
}

void lightControl(arduinoState state, boneState bone) {
	static long lastChangeTime = millis();
	bool flashState = false;
	uint8_t pixelCounter;
	uint32_t color0;
	
	switch (state) {
		case BOAT_POWERUP:
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
		case BOAT_ARMED:
			color0 = blu;
			break;
		case BOAT_SELFTEST:
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
		case BOAT_DISARMED:
			color0 = amb;
			break;
		case BOAT_ACTIVE:
			color0 = grn;
			break;
		case BOAT_LOWBATTERY:
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
		case BOAT_SELFRECOVERY:
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
		case BOAT_ACTIVERUDDER:
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
		case BOAT_NONE:
		default:
			color0 = 0;
	}
	for (pixelCounter = 0; pixelCounter < ardLightCount; pixelCounter++) {
		ardLights.setPixelColor(pixelCounter, color0);
	}
	
	switch (bone) {
		case BONE_START:
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
		case BONE_SELFTEST:
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
		case BONE_DISARMED:
			color0 = amb;
			break;
		case BONE_FAULT:
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
		case BONE_ARMED:
			color0 = blu;
			break;
		case BONE_MANUAL:
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
		case BONE_WAYPOINT:
			color0 = grn;
			break;
		case BONE_NOSIGNAL:
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
		case BONE_RETURN:
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
		case BONE_ARMEDTEST:
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
		case BONE_UNKNOWN:
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

///////////////////////////////
// REST function definitions //
///////////////////////////////

int writeBoneState (String params) {
	for (uint8_t i = 0; i < boneStateCount; i++) {
		if(params.equals(boneStates[i])) {
			boat.bone = (boneState)i;
			boat.boneStateString = boneStates[i];
			return 0;
		}
	}
	boat.bone = BONE_UNKNOWN;
	boat.boneStateString = boneStates[BONE_UNKNOWN];
	return -1;
}

int writeCommand (String params) {
	for (uint8_t i = 0; i < arduinoStateCount; i++) {
		if(params.equals(arduinoStates[i])) {
			boat.command = (arduinoState)i;
			boat.commandString = arduinoStates[i];
			return 0;
		}
	}
	boat.command = BOAT_NONE;
	boat.commandString = arduinoStates[BOAT_NONE];
	return -1;
}

int writeThrottle (String params) {
	uint8_t throttleIn = params.toInt();
	if ((boat.state == BOAT_ACTIVE) || (boat.state == BOAT_ACTIVERUDDER)) {
		if (((throttleIn >= -1) && (throttleIn <= 5)) || 
			(throttleIn == -3) || (throttleIn == -5)) {
				boat.throttle = (throttleState)throttleIn;
		}
	}
	return boat.throttle;
}

int writeHeadingTarget (String params) {
	double headingIn = params.toFloat();
	if (boat.state == BOAT_ACTIVE) {
		if ((headingIn <= 360) || (headingIn >= 0)) {
			boat.headingTarget = headingIn;
		}
	}
	return (boat.headingTarget * 10);
}

int writeHeadingDelta (String params) {
	double headingIn = params.toFloat();
	if (boat.state == BOAT_ACTIVE) {
		if ((headingIn <= 180) || (headingIn >= -180)) {
			boat.headingTarget += headingIn;
			if (boat.headingTarget < 0) boat.headingTarget += 360;
			if (boat.headingTarget > 360) boat.headingTarget -= 360;
		}
	}
	return (boat.headingTarget * 10);
}

int writeRudder (String params) {
	double rudderIn = params.toFloat();
	if ((boat.state == BOAT_ARMEDTEST) || 
		(boat.state == BOAT_ACTIVERUDDER) ||
		(boat.state == BOAT_ARMED)) {
			if ((rudderIn <= pidMax) || (rudderIn >= pidMin)) {
				boat.rudder = rudderIn;
			}
	}
	return (boat.rudder * 10);
}

int writeKp (String params) {
	double Kin = params.toFloat();
	if ((Kin <= 10) || (Kin >= -10)) {
		boat.Kp = Kin;
		steeringPID.SetTunings(boat.Kp, boat.Ki, boat.Kd);
	}
	return (boat.Kp * 1000);
}

int writeKi (String params) {
	double Kin = params.toFloat();
	if ((Kin <= 10) || (Kin >= -10)) {
		boat.Ki = Kin;
		steeringPID.SetTunings(boat.Kp, boat.Ki, boat.Kd);
	}
	return (boat.Ki * 1000);
}

int writeKd (String params) {
	double Kin = params.toFloat();
	if ((Kin <= 10) || (Kin >= -10)) {
		boat.Kd = Kin;
		steeringPID.SetTunings(boat.Kp, boat.Ki, boat.Kd);
	}
	return (boat.Kd * 1000);
}

int writeHorn (String params) {
	if (boat.state == BOAT_ARMEDTEST) {
		if (params.equals("ON")) {
			boat.horn = HIGH;
			return 1;
		} else if (params.equals("OFF")) {
			boat.horn = LOW;
			return 0;
		}
	}
	return -1;
}

int writeMotorDirRly (String params) {
	if (boat.state == BOAT_ARMEDTEST) {
		if (params.equals("ON")) {
			boat.motorDirRly = HIGH;
			return 1;
		} else if (params.equals("OFF")) {
			boat.motorDirRly = LOW;
			return 0;
		}
	}
	return -1;
}

int writeMotorWhtRly (String params) {
	if (boat.state == BOAT_ARMEDTEST) {
		if (params.equals("ON")) {
			boat.motorWhtRly = HIGH;
			return 1;
		} else if (params.equals("OFF")) {
			boat.motorWhtRly = LOW;
			return 0;
		}
	}
	return -1;
}

int writeMotorYlwRly (String params) {
	if (boat.state == BOAT_ARMEDTEST) {
		if (params.equals("ON")) {
			boat.motorYlwRly = HIGH;
			return 1;
		} else if (params.equals("OFF")) {
			boat.motorYlwRly = LOW;
			return 0;
		}
	}
	return -1;
}

int writeMotorRedRly (String params) {
	if (boat.state == BOAT_ARMEDTEST) {
		if (params.equals("ON")) {
			boat.motorRedRly = HIGH;
			return 1;
		} else if (params.equals("OFF")) {
			boat.motorRedRly = LOW;
			return 0;
		}
	}
	return -1;
}

int writeMotorRedWhtRly (String params) {
	if (boat.state == BOAT_ARMEDTEST) {
		if (params.equals("ON")) {
			boat.motorRedWhtRly = HIGH;
			return 1;
		} else if (params.equals("OFF")) {
			boat.motorRedWhtRly = LOW;
			return 0;
		}
	}
	return -1;
}

int writeMotorRedYlwRly (String params) {
	if (boat.state == BOAT_ARMEDTEST) {
		if (params.equals("ON")) {
			boat.motorRedYlwRly = HIGH;
			return 1;
		} else if (params.equals("OFF")) {
			boat.motorRedYlwRly = LOW;
			return 0;
		}
	}
	return -1;
}

int writeServoPower (String params) {
	if (boat.state == BOAT_ARMEDTEST) {
		if (params.equals("ON")) {
			boat.servoPower = HIGH;
			return 1;
		} else if (params.equals("OFF")) {
			boat.servoPower = LOW;
			return 0;
		}
	}
	return -1;
}

int	boneHeartBeat (String params) {
	boat.timeOfLastBoneHB = millis();
	return 0;
}

int	shoreHeartBeat (String params) {
	boat.timeOfLastShoreHB = millis();
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
	return BOAT_SELFTEST;
}

arduinoState executeSelfTest (boatVector * thisBoat, arduinoState lastState) {
	static uint8_t faultCnt = 0;
	static double headingRef;
	static long startTime = millis();
	static uint8_t headingFaultCnt = 0;
	static uint8_t accelFaultCnt = 0;
	static uint8_t gyroFaultCnt = 0;
	static uint8_t signalFaultCnt = 0;
	arduinoState myState = BOAT_DISARMED;
	throttleState myThrottle = STOP;

	LogSerial.println("**** Self-testing... ****");

	// if we've just entered this state, reset all the counters
	if (lastState != BOAT_SELFTEST) {
		faultCnt = 0; 
		headingRef = thisBoat->orientation.heading;
		headingFaultCnt = 0;
		accelFaultCnt = 0;
		gyroFaultCnt = 0;
		signalFaultCnt = 0;
		startTime = millis();
	}
	
	// Power down all relays, power up the servo, and center the rudder
	thisBoat->motorDirRly 		= LOW;
	thisBoat->motorWhtRly 		= LOW;
	thisBoat->motorRedRly 		= LOW;
	thisBoat->motorYlwRly 		= LOW;
	thisBoat->motorRedWhtRly 	= LOW;
	thisBoat->motorRedYlwRly 	= LOW;
	thisBoat->rudder 			= 0;
	thisBoat->servoPower		= HIGH;
	
	// if we're commanded into ArmedTest, go there
	if (thisBoat->command == BOAT_ARMEDTEST) {
		thisBoat->command = BOAT_NONE;
		return BOAT_ARMEDTEST;
	}
	
	// check the battery
	if (thisBoat->internalVoltage < testVoltageLimit) {
		faultCnt++;
		thisBoat->faultString |= FAULT_LOW_BAT;
	}
	
	// check the orientation
	if ((millis() - startTime) < sensorTestPeriod) {
		if ((thisBoat->orientation.roll > tiltDeviationLimit) || (thisBoat->orientation.pitch > tiltDeviationLimit)) {
			faultCnt++;
			LogSerial.print("Sensor outside of roll/pitch limits. Measure values roll: ");
			LogSerial.print(thisBoat->orientation.roll);
			LogSerial.print(" pitch: ");
			LogSerial.println(thisBoat->orientation.pitch);
			thisBoat->faultString |= FAULT_SENSOR;
		}
		if (abs(getHeadingError(thisBoat->orientation.heading, headingRef)) > compassDeviationLimit) {
			LogSerial.print("Compass outside of deviation limits. Compass heading: ");
			LogSerial.print(thisBoat->orientation.heading);
			LogSerial.print(" Reference: ");
			LogSerial.print(headingRef);
			LogSerial.print(" Error: ");
			LogSerial.println(getHeadingError(thisBoat->orientation.heading, headingRef));
			faultCnt++;
			thisBoat->faultString |= FAULT_SENSOR;
		}
	}
	
	// check for incoming signal
	if ((millis() - thisBoat->timeOfLastPacket) > signalTestPeriod) {
		faultCnt++;
		thisBoat->faultString |= FAULT_NO_SIGNAL;
		LogSerial.print("Signal timeout. Current time: ");
		LogSerial.print(millis());
		LogSerial.print(" Last time: ");
		LogSerial.println(thisBoat->timeOfLastPacket);
	} else {
		if (thisBoat->faultString & FAULT_NO_SIGNAL) {
			thisBoat->faultString &= !FAULT_NO_SIGNAL;
			faultCnt--;
		}
		LogSerial.print("Removing signal timeout. Current time: ");
		LogSerial.print(millis());
		LogSerial.print(" Last time: ");
		LogSerial.print(thisBoat->timeOfLastPacket);
		LogSerial.print(" Fault string: ");
		LogSerial.println(thisBoat->faultString);
	}
	
	// Check for fault from the Beaglebone
	if (BONE_FAULT == thisBoat->bone) {
		faultCnt++;
		thisBoat->faultString |= FAULT_BB_FAULT;
	}
	
	// Check for the end of the test
	if ((millis() - startTime) > startupTestPeriod) {
		if (faultCnt) {
			LogSerial.print("Got faults on startup. Fault string: ");
			LogSerial.println(thisBoat->faultString, HEX);
			return BOAT_FAULT;
		} else {
			return BOAT_DISARMED;
		}
	}
	
	return BOAT_SELFTEST;
}

arduinoState executeDisarmed (boatVector * thisBoat, arduinoState lastState) {
	static bool lastEnbButton = false;
	static long startEnbTime = millis();

	LogSerial.println("**** Disarmed ****");
	
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
	if (thisBoat->enbButton && !(lastEnbButton)) {
		startEnbTime = millis();
	} else if (!(thisBoat->enbButton)) {
		startEnbTime = millis();
	}
	lastEnbButton = thisBoat->enbButton;
   
	// determine next state
	if ((millis() - startEnbTime) > enbButtonTime) return BOAT_ARMED;
	if (BONE_FAULT == thisBoat->bone) {
		thisBoat->faultString |= FAULT_BB_FAULT;
		return BOAT_FAULT;
	}
	if ((millis() - thisBoat->timeOfLastBoneHB) > disarmedPacketTimeout) {
		thisBoat->faultString |= FAULT_NO_SIGNAL;
		return BOAT_FAULT;
	}
	return BOAT_DISARMED;
}

arduinoState executeFault (boatVector * thisBoat, arduinoState lastState) {

	LogSerial.println("**** Fault ****");
	
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
	
	LogSerial.print("Fault string: ");
	LogSerial.println(thisBoat->faultString, HEX);
	if (0 == thisBoat->faultString) return BOAT_DISARMED;
	if (BOAT_SELFTEST == thisBoat->command) return BOAT_SELFTEST;

	return BOAT_FAULT;
  
}

arduinoState executeArmed (boatVector * thisBoat, arduinoState lastState) {
	static long startStopTime = millis();
	static long startStateTime = millis();
	static arduinoState originState = BOAT_DISARMED;

	LogSerial.println("**** Armed ****");
	
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
	if (lastState != BOAT_ARMED) {
		startStateTime 			= millis();
		originState 			= lastState;
		thisBoat->horn 			= HIGH;
	}
	
	// check for the stop button
	if (!(thisBoat->stopButton)) {	
		// This resets the timer every time this state executes without the button pressed
		// As soon as the button is pressed, this is no longer reset and the timer runs.
		startStopTime = millis();
	}
	if ((millis() - startStopTime) > stopButtonTime) return BOAT_DISARMED;
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		thisBoat->faultString |= FAULT_LOW_BAT;
		thisBoat->horn = LOW;
		return BOAT_LOWBATTERY;
	}
	
	// check for packet timeout
	if ((millis() - thisBoat->timeOfLastBoneHB) > armedPacketTimeout) {
		thisBoat->faultString |= FAULT_NO_SIGNAL;
		thisBoat->horn = LOW;
		return BOAT_FAULT;
	}
	
	// Check to see if we came from a safe state and whether or not the horn timeout is over.
	// If we came from a safe state and the horn timeout is not over, sound the horn and reject
	// commands. 
	if (((millis() - startStateTime) < hornTimeout) && (BOAT_DISARMED == originState)) {
		thisBoat->horn = HIGH;
		return BOAT_ARMED;
	} else {
		thisBoat->horn = LOW;
		if (BOAT_DISARMED == thisBoat->command) {
			thisBoat->command = BOAT_NONE;
			return BOAT_DISARMED;
		}
		if (BOAT_ACTIVE == thisBoat->command) {
			thisBoat->command = BOAT_NONE;
			return BOAT_ACTIVE;
		}
		if (BOAT_ACTIVERUDDER == thisBoat->command) {
			thisBoat->command = BOAT_NONE;
			return BOAT_ACTIVERUDDER;
		}
	}
	
}

arduinoState executeArmedTest (boatVector * thisBoat, arduinoState lastState) {
	static long startStopTime = millis();
	static long startStateTime = millis();

	LogSerial.println("**** ArmedTest ****");
	
	// reset timers & relays upon state entry
	if (lastState != BOAT_ARMEDTEST) {
		startStateTime 				= millis();
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
		startStopTime = millis();
	}
	if ((millis() - startStopTime) > stopButtonTime) {		
		restInput.setEnable(AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
		return BOAT_DISARMED;
	}
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		thisBoat->faultString |= FAULT_LOW_BAT;
		thisBoat->horn = LOW;
		restInput.setEnable(AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
		return BOAT_FAULT;
	}
	
	// check for packet timeout
	if ((millis() - thisBoat->timeOfLastShoreHB) > armedPacketTimeout) {
		thisBoat->faultString |= FAULT_NO_SIGNAL;
		thisBoat->horn = LOW;
		restInput.setEnable(AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
		return BOAT_FAULT;
	}
	
	// Check to see if we came from a safe state and whether or not the horn timeout is over.
	// If we came from a safe state and the horn timeout is not over, sound the horn and reject
	// commands. 
	if ((millis() - startStateTime) < hornTimeout) {
		thisBoat->horn = HIGH;
		return BOAT_ARMEDTEST;
	} else {
		thisBoat->horn = LOW;
		restInput.setEnable(AREST_ENB_DIGITAL | AREST_ENB_ANALOG | AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
		if (BOAT_DISARMED == thisBoat->command) {
			thisBoat->command = BOAT_NONE;
			restInput.setEnable(AREST_ENB_VARIABLE | AREST_ENB_FUNCTION);
			return BOAT_DISARMED;
		}
		
	}
	
}

arduinoState executeActive (boatVector * thisBoat, arduinoState lastState) {
	static long startStopTime = millis();

	LogSerial.println("**** Active ****");
	
	if (lastState != BOAT_ACTIVE) {
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
		startStopTime = millis();
	}
	if ((millis() - startStopTime) > stopButtonTime) {		
		steeringPID.SetMode(MANUAL);
		return BOAT_DISARMED;
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
		thisBoat->faultString |= FAULT_LOW_BAT;
		steeringPID.SetMode(MANUAL);
		return BOAT_LOWBATTERY;
	}
	
	// check for packet timeout
	if ((millis() - thisBoat->timeOfLastBoneHB) > armedPacketTimeout) {
		thisBoat->faultString |= FAULT_NO_SIGNAL;
		return BOAT_SELFRECOVERY;
	}
	
	// check for commands
	if (BOAT_ACTIVERUDDER == thisBoat->command) {
		thisBoat->command = BOAT_NONE;
		steeringPID.SetMode(MANUAL);
		return BOAT_ACTIVERUDDER;
	}
	if (BOAT_DISARMED == thisBoat->command) {
		thisBoat->command = BOAT_NONE;
		steeringPID.SetMode(MANUAL);
		return BOAT_DISARMED;
	}
}

arduinoState executeActiveRudder (boatVector * thisBoat, arduinoState lastState)  {
	static long startStopTime = millis();

	LogSerial.println("**** ActiveRudder ****");
	
	if (lastState != BOAT_ACTIVERUDDER) {
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
		startStopTime = millis();
	}
	if ((millis() - startStopTime) > stopButtonTime) {	
		return BOAT_DISARMED;
	}
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		thisBoat->faultString |= FAULT_LOW_BAT;
		return BOAT_LOWBATTERY;
	}
	
	// check for packet timeout
	if ((millis() - thisBoat->timeOfLastShoreHB) > armedPacketTimeout) {
		thisBoat->faultString |= FAULT_NO_SIGNAL;
		return BOAT_SELFRECOVERY;
	}
	
	// check for commands
	if (BOAT_ACTIVE == thisBoat->command) {
		thisBoat->command = BOAT_NONE;
		return BOAT_ACTIVE;
	}
	if (BOAT_DISARMED == thisBoat->command) {
		thisBoat->command = BOAT_NONE;
		return BOAT_DISARMED;
	}
}

arduinoState executeLowBattery (boatVector * thisBoat, arduinoState lastState) {
	static arduinoState originState = BOAT_LOWBATTERY;
	static long startStopTime = millis();

	Serial.println("**** Low Battery ****");
	
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
	if (BOAT_LOWBATTERY != lastState) {
		if ((BOAT_ARMED == lastState) || 
			(BOAT_ACTIVE == lastState) ||
			(BOAT_ACTIVERUDDER == lastState) ||
			(BOAT_SELFRECOVERY == lastState)) {
				originState = lastState;
		} else originState = BOAT_DISARMED;
	}
	
	// check for the stop button
	if (!(thisBoat->stopButton)) {	
		// This resets the timer every time this state executes without the button pressed
		// As soon as the button is pressed, this is no longer reset and the timer runs.
		startStopTime = millis();
	}
	if ((millis() - startStopTime) > stopButtonTime) {	
		return BOAT_DISARMED;
	}
	
	// check for battery voltage
	if (thisBoat->internalVoltage > recoverVoltageLimit) {
		thisBoat->faultString &= ~FAULT_LOW_BAT;
		return originState;
	}

	return BOAT_LOWBATTERY;
}

arduinoState executeSelfRecovery (boatVector * thisBoat, arduinoState lastState) {
	static arduinoState originState = BOAT_SELFRECOVERY;
	static long startStopTime = millis();

	LogSerial.println("**** SelfRecovery ****");
	
	if (lastState != BOAT_SELFRECOVERY) {
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
		startStopTime = millis();
	}
	if ((millis() - startStopTime) > stopButtonTime) {		
		steeringPID.SetMode(MANUAL);
		return BOAT_DISARMED;
	}
	
	// check for low voltage
	if (thisBoat->internalVoltage < serviceVoltageLimit) {
		thisBoat->faultString |= FAULT_LOW_BAT;
		steeringPID.SetMode(MANUAL);
		return BOAT_LOWBATTERY;
	}
	
	// figure out what state we will return to 
	if (BOAT_SELFRECOVERY != lastState) {
		if ((BOAT_ARMED == lastState) || 
			(BOAT_ACTIVE == lastState) ||
			(BOAT_ACTIVERUDDER == lastState)) {
				originState = lastState;
		} else originState = BOAT_DISARMED;
	}
	
	// check for heartbeat...
	if ((millis() - thisBoat->timeOfLastBoneHB) < armedPacketTimeout) {
		steeringPID.SetMode(MANUAL);
		return originState;
	}
	
	return BOAT_SELFRECOVERY;
}

