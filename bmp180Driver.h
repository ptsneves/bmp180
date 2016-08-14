#ifndef BMP180DRIVER_H
#define BMP180DRIVER_H

#include <math.h> //Users need to know about NaN

#define BMP180_EEPROM_WORDS 11 //BMP180@Page13 [..]partitioned in 11 words [...]

/*
 * BMP180:Page12
 *
 */
typedef enum {
	NO_SELECTION = 0,
	TEMPERATURE = 0x2e,
	PRESSURE_ULTRA_LOW_POWER = 0x34,
	PRESSURE_STANDARD = 0x74,
	PRESSURE_HIGH_RESOLUTION = 0xB4,
	PRESSURE_ULTRA_HIGH_RESOLUTION = 0xF4
} MeasureMode;

typedef struct {
	uint8_t device_address; //i2c device address
	int16_t eeprom_coeff[BMP180_EEPROM_WORDS]; //correction coefficients
	MeasureMode measure_mode; //See MeasureMode
} BMP180Data;


/*
 * Read temperature in degrees Celcius.
 * If any error occurs the function will return a NAN. Use isnan
 *  to verify this condition. Failure happens if the device is
 * not connected, is not valid or if bmp180 pointer is NULL.
 */
double readTemperature(BMP180Data * const bmp180); // unit=C

/*
 * Read pressure in hPa. If any error occurs
 * If any error occurs the function will return a NAN. Use isnan
 *  to verify this condition. Failure happens if the device is
 * not connected, is not valid or if bmp180 pointer is NULL.
 */
double readPressure(BMP180Data * const bmp180); // unit=hPa

#endif