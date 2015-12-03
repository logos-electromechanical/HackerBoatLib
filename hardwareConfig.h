
#ifndef HACKERBOATLIB_HDW_h
#define HACKERBOATLIB_HDW_h

// steering and servo constants
#define Kp_start				5.0
#define Ki_start				0.25
#define Kd_start				0.0
#define pidMax					100.0
#define pidMin					-100.0
#define servoMin				860
#define servoMax				2240
#define servoCenter				((servoMin + servoMax)/2)

// test limits
#define compassDeviationLimit	10.0	/**< Limit of compass swing, in degrees, during test period 	*/
#define tiltDeviationLimit		15.0	/**< Limit of tilt in degrees from horizontal during test		*/
#define rateDeviationLimit		15.0	/**< Gyro rate limit, in degrees. Currently unused. 			*/
#define testVoltageLimit 		12.0	/**< Battery voltage lower limit during powerup test			*/
#define serviceVoltageLimit		10.0	/**< Battery voltage lower limit in service						*/
#define recoverVoltageLimit 	11.0	/**< Battery voltage required to recover from low battery state	*/

// time delays
#define sensorTestPeriod  	    5000	/**< Period to check for sensor deviations, in ms 							*/
#define signalTestPeriod      	6000	/**< Period to wait for Beaglebone signal 									*/
#define startupTestPeriod     	6500	/**< Period to stay in the self-test state 									*/
#define enbButtonTime        	5000	/**< Time the enable button needs to be pressed, in ms, to arm the boat		*/
#define stopButtonTime       	250		/**< Time the stop button needs to be pressed, in ms, to disarm the boat	*/
#define disarmedPacketTimeout 	60000	/**< Connection timeout, in ms, in the disarmed state						*/
#define armedPacketTimeout   	60000	/**< Connection timeout, in ms, in the armed state							*/
#define activePacketTimeout  	300000	/**< Connection timeout, in ms, in the active state							*/
#define hornTimeout  			2000	/**< Time in ms to sound the horn for before entering an unsafe state		*/	
#define sendDelay            	1000	/**< Time in ms between packet transmissions 								*/
#define flashDelay  			500		/**< Time in ms between light transitions while flashing					*/

// port mapping
static HardwareSerial LogSerial = 		Serial;		/**< Serial port used for logging and feedback */
static HardwareSerial RESTSerial = 		Serial1;	/**< Serial port used for REST commands and response */

// pin mapping
#define servoEnable          	2		/**< Enable pin for the steering servo power supply 	*/
#define steeringPin          	3		/**< Steering servo control pin							*/
#define internalBatVolt      	A0		/**< Internal battery voltage pin						*/
#define batteryVolt 			A1		/**< External battery voltage							*/
#define batteryCurrent 			A10		/**< External battery current							*/
#define motorVolt 				A15		/**< Motor voltage (measured at speed control input)	*/
#define motorCurrent 			A13		/**< Motor current (measured at speed control input)	*/
#define relayDir              	52		/**< Pin to control motor direction. LOW = forward, HIGH = reverse 	*/
#define relayDirFB            	53		/**< Motor direction relay wraparound pin				*/
#define relaySpeedWht         	51		/**< Motor relay white									*/
#define relaySpeedWhtFB       	50		/**< Motor relay white wraparound						*/
#define relaySpeedYlw         	48  	/**< Motor relay yellow									*/
#define relaySpeedYlwFB       	49		/**< Motor relay yellow wraparound						*/
#define relaySpeedRed         	47  	/**< Motor relay red									*/
#define relaySpeedRedFB       	46		/**< Motor relay red wraparound							*/
#define relaySpeedRedWht  	 	44		/**< Red-White motor crossover relay					*/
#define relaySpeedRedWhtFB 	 	45		/**< Red-White motor crossover relay wraparound			*/
#define relaySpeedRedYlw  	 	43		/**< Red-Yellow motor crossover relay					*/
#define relaySpeedRedYlwFB 	 	42		/**< Red-Yellow motor crossover relay wraparound		*/
#define hornPin				 	40		/**< Alert horn 										*/
#define hornPinFB 				41 		/**< Alert horn wraparound								*/
#define arduinoLightsPin  		39		/**< Arduino state indicator lights pin					*/
#define boneLightsPin 			38		/**< Beaglebone state indicator lights pin				*/
#define enableButtonPin			37		/**< Enable button input								*/	
#define stopButtonPin 			36		/**< Stop button input									*/

// pin-associated constants
/**< The number of pixels in the BeagleBone light strip	*/	
#define boneLightCount 			8		
/**< The number of pixels in the Arduino light strip	*/
#define ardLightCount 			8		

// color definitions
/**< pixel colors for green		*/
#define grn 	Adafruit_NeoPixel::Color(0, 0xff, 0)		
/**< pixel colors for red		*/
#define red 	Adafruit_NeoPixel::Color(0xff, 0, 0)
/**< pixel colors for blue		*/		
#define blu 	Adafruit_NeoPixel::Color(0, 0, 0xff)	
/**< pixel colors for amber		*/	
#define amb 	Adafruit_NeoPixel::Color(0xff, 0xbf, 0)	
/**< pixel colors for white		*/	
#define wht 	Adafruit_NeoPixel::Color(0xff, 0xff, 0xff)		

//const uint32_t lightTimeout =        1490000;
//const uint32_t ctrlTimeout =         1500000;
//const uint16_t lowVoltCutoff =       750;
//const uint8_t relayAux1 =      47;

#endif