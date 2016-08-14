#include <stdint.h>
#include <stddef.h>
#include "bmp180Driver.h"

#define CHIP_ID 0x55

typedef uint8_t BOOL;

/*
 * BMP180@Page18 Figure 6 has a typo in the bit columns names like ADC_OUT
 * i2c registers
 */
typedef enum {
    REGISTER_OUT_MSB = 0xF6,
    REGISTER_OUT_LSB = 0xF7,
    REGISTER_OUT_XLSB = 0xF8,
    REGISTER_CTRL_MEAS = 0xF4,
    REGISTER_CHIP_ID = 0xD0,
    REGISTER_EEPROM_START = 0xAA
} RegisterAddress;

/*
 * eeprom_coeff indexes for pressure calculation
 */
typedef enum {
    EEPROM_AC1 = 0,
    EEPROM_AC2,
    EEPROM_AC3,
    EEPROM_AC4,
    EEPROM_AC5,
    EEPROM_AC6,
    EEPROM_B1,
    EEPROM_B2,
    EEPROM_MB,
    EEPROM_MC,
    EEPROM_MD,
    EEPROM_END_COEFFICIENTS = BMP180_EEPROM_WORDS
} EepromCoefficients;

typedef struct {
    int32_t UT;
    int32_t X1;
    int32_t X2;
    int32_t B5;
    int32_t raw_temperature;
    BOOL valid;
} RawTemperatureData;

// I2C burst read (reads from 7 bit I2C device 'address' 'size' bytes
//starting with register 'reg' into 'data')
extern void i2c_read(uint8_t address, uint8_t reg, uint8_t *data, size_t size);
// I2C burst write (writes to 7 but I2C device 'address' 'size' bytes,
//starting with register 'reg')

extern void i2c_write(uint8_t address, uint8_t reg, uint8_t *data, size_t size);

extern void usleep(uint32_t microsseconds);

/*
 * Returns the power of 2 in a type agnostic way.
 */
#define POW2(exp) (1 << exp)

/*
 * A mode validation function.
 * Returns whether the measuring mode is indeed part of the enum
 */
static BOOL isValidMode(MeasureMode mode) {
    switch (mode) {
        case TEMPERATURE:
        case PRESSURE_ULTRA_LOW_POWER:
        case PRESSURE_STANDARD:
        case PRESSURE_HIGH_RESOLUTION:
        case PRESSURE_ULTRA_HIGH_RESOLUTION:
            return 1;
            break;
        default:
            break;
    }
    return 0;
}

/*
 * BMP180@Page18: Chip-id (register D0h): This value is fixed to 0x55 and
 * can be used to check whether communication is functioning.
 * Return: 0 on false
 */
static BOOL isConncected(BMP180Data * const bmp180) {
    uint8_t chip_id = 0;
    i2c_read(bmp180->device_address, REGISTER_CHIP_ID, &chip_id,
            sizeof (chip_id));

    return (chip_id == 0x55);
}

/*
 * BMP180@Page13:The data communication can be checked by checking that
 * none of the words has the value 0 or 0xFFFF.
 * Return: 0 on false
 */
static BOOL isValidEEPROMWord(uint16_t word) {
    return (word != 0xFFFF && word != 0);
}

/*
 * Populates the eeprom_coeff membmer of bmp180. It only fecthes the
 * the coefficients from the chip if bmp180 does not have the
 * coefficients already.
 * Returns 1 if all the coefficients were correctly read or available.
 */
static BOOL readEEPROMCoefficients(BMP180Data * const bmp180) {
    uint8_t coeff_address = REGISTER_EEPROM_START;
    EepromCoefficients i;
    for (i = EEPROM_AC1; i < EEPROM_END_COEFFICIENTS; i++) {
        if (isValidEEPROMWord(bmp180->eeprom_coeff[i])) {
            coeff_address += 2;
        }
        else {
            uint8_t i2c_data[2] = {};

            i2c_read(bmp180->device_address, coeff_address,
                    i2c_data, 2);
            coeff_address += 2;
            bmp180->eeprom_coeff[i] = (i2c_data[0] << 8) + i2c_data[1];

            if (!isValidEEPROMWord(bmp180->eeprom_coeff[i]))
                break;
        }

    }
    return coeff_address == (0xBE + 2); //BF+1 due to the last increment
}

/*
 * Makes the bmp180 sensor start ADC convertions and waits for them to
 * be ready according to bmp180:Page21 Table 8.
 * Returns 1 if the request was possible.
 */
static BOOL requestMeasurement(BMP180Data * const bmp180) {
    /*
     *  In C the MeasureMode is not guaranteed to actually be enum,
     * just an int. We want to guarantee we don't use unsafe input
     * as register input
     */
    if (!isValidMode(bmp180->measure_mode))
        return 0;

    if (!isConncected(bmp180))
        return 0;

    if (!readEEPROMCoefficients(bmp180))
        return 0;

    i2c_write(bmp180->device_address, REGISTER_CTRL_MEAS,
            (uint8_t*) & bmp180->measure_mode, 1);

    //BMP180:Page21: Table 8
    uint16_t delay = 0U;
    switch (bmp180->measure_mode) {
        case TEMPERATURE:
        case PRESSURE_ULTRA_LOW_POWER:
            delay = 4500U;
            break;
        case PRESSURE_STANDARD:
            delay = 7500U;
            break;
        case PRESSURE_HIGH_RESOLUTION:
            delay = 13500U;
            break;
        case PRESSURE_ULTRA_HIGH_RESOLUTION:
        default: //unreachable label
            delay = 25500;
            break;
    }
    usleep(delay);
    return 1;
}

/*
 * Internal function that gathers the temperature information
 * alongside with coefficient information that can be used by
 * readPressure and readTemperature.
 * Returns RawTemperatureData, which may be valid or not according
 * to valid data member. It has to be checked.
 */
RawTemperatureData readRawTemperature(BMP180Data * const bmp180) {
    uint8_t i2c_data[2] = {0};
    RawTemperatureData data = {};

    if (!requestMeasurement(bmp180))
        return data;

    i2c_read(bmp180->device_address, REGISTER_OUT_MSB, i2c_data, 2);
    data.UT = (i2c_data[0] << 8) + i2c_data[1];

    //TODO: Check valid values??

    int16_t *coeff = bmp180->eeprom_coeff;
    uint16_t *u_coeff = (uint16_t*) coeff; //forcing to read these as unsigned

    data.X1 = (data.UT - u_coeff[EEPROM_AC6]) * u_coeff[EEPROM_AC5] / POW2(15);

    data.X2 = coeff[EEPROM_MC] * POW2(11) / (data.X1 + coeff[EEPROM_MD]);
    data.B5 = data.X1 + data.X2;
    data.raw_temperature = (data.B5 + 8) / POW2(4);
    data.valid = 1;
    return data;
}

double readTemperature(BMP180Data * const bmp180) {
    double temperature = NAN;
    if (bmp180 == NULL) {
        return NAN;
    }

    if (bmp180->measure_mode != TEMPERATURE)
        bmp180->measure_mode = TEMPERATURE;
    RawTemperatureData raw_data = readRawTemperature(bmp180);

    if (raw_data.valid)
        temperature = raw_data.raw_temperature / 10.0;

    return temperature;
}

double readPressure(BMP180Data * const bmp180) {
    if (bmp180 == NULL) {
        return NAN;
    }

    int32_t UP;
    uint8_t i2c_data[3] = {};
    int32_t X1, X2, X3, B3, B6, p;
    uint32_t B4, B7;
    MeasureMode original_mode = bmp180->measure_mode;

    bmp180->measure_mode = TEMPERATURE;
    RawTemperatureData raw_temperature = readRawTemperature(bmp180);

    if (!raw_temperature.valid) {
        return NAN;
    }

    bmp180->measure_mode = original_mode;
    if (!requestMeasurement(bmp180))
        return NAN;


    int16_t *coeff = bmp180->eeprom_coeff;
    uint16_t *u_coeff = (uint16_t*) coeff;
    uint8_t oss = bmp180->measure_mode >> 6;

    i2c_read(bmp180->device_address, REGISTER_OUT_MSB, i2c_data, 3);
    UP = ((i2c_data[0] << 16) + (i2c_data[1] << 8) + i2c_data[2]) >>
            (8 - oss);

    B6 = raw_temperature.B5 - 4000;
    X1 = (coeff[EEPROM_B2] * (B6 * B6 / POW2(12))) / POW2(11);
    X2 = coeff[EEPROM_AC2] * B6 / POW2(11);
    X3 = X1 + X2;
    B3 = ((((int32_t) coeff[EEPROM_AC1] * 4 + X3) << oss)
            + 2) / 4;
    X1 = coeff[EEPROM_AC3] * B6 / POW2(13);
    X2 = (coeff[EEPROM_B1] * (B6 * B6 / POW2(12))) / POW2(16);
    X3 = ((X1 + X2) + 2) / POW2(2);
    B4 = u_coeff[EEPROM_AC4] * (uint32_t) (X3 + 32768) / POW2(15);
    B7 = ((uint32_t) UP - B3) * (50000 >> oss);

    if (B7 < 0x80000000)
        p = (B7 * 2) / B4;
    else
        p = (B7 / B4) * 2;

    X1 = (p / POW2(8)) * (p / POW2(8));
    X1 = (X1 * 3038) / POW2(16);
    X2 = (-7357 * p) / POW2(16);
    p = p + (X1 + X2 + 3791) / POW2(4);
    return (double) p / 100.0;
}