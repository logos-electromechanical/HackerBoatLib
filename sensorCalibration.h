#ifndef HACKERBOATLIB_CAL_h
#define HACKERBOATLIB_CAL_h

// calibration constants
// these constants are used to calibrate the accelerometer and magnetometer
#define X_ACCEL_OFFSET          (0)
#define X_ACCEL_GAIN            (0.1)
#define Y_ACCEL_OFFSET          (0)
#define Y_ACCEL_GAIN            (0.1)
#define Z_ACCEL_OFFSET          (0)
#define Z_ACCEL_GAIN            (0.1)
#define X_MAG_OFFSET            (-39.59)
#define X_MAG_GAIN              (0.933)
#define Y_MAG_OFFSET            (10.82)
#define Y_MAG_GAIN              (0.943)
#define Z_MAG_OFFSET            (-23.16)
#define Z_MAG_GAIN              (1.054)

const double motorCurrentMult =			1.0;	/**< Motor current gain									*/
const double motorVoltMult = 			1.0;	/**< Motor voltage gain									*/
const int16_t batteryCurrentOffset =	0;		/**< Battery current offset								*/
const double batteryCurrentMult =		1.0;	/**< Battery current gain								*/
const double internalBatVoltMult =   	1.0;	/**< Internal battery voltage multiplier				*/
const double batteryVoltMult = 			1.0;	/**< Battery voltage gain								*/
const int16_t motorCurrentOffset = 		0;		/**< Motor current offset								*/

#endif