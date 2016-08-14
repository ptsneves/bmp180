#include <string.h>
#include <stdio.h>
#include "bmp180Simulator.h"

typedef enum {
    AC1 = 0xAA,
    AC2 = 0xAC,
    AC3 = 0xAE,
    AC4 = 0xB0,
    AC5 = 0xB2,
    AC6 = 0xB4,
    B1 = 0xB6,
    B2 = 0xB8,
    MB = 0xBA,
    MC = 0xBC,
    MD = 0xBE,
    OUT_MSB = 0xF6,
    OUT_LSB = 0xF7,
    OUT_XLSB = 0xF8,
    CTRL_MEAS = 0xF4,
    CHIP_ID = 0xD0
} REGISTERS;

typedef enum {
    NONE_SELECTED = 0,
    TEMPERATURE = 0x2e,
    PRESSURE_ULTRA_LOW_POWER = 0x34,
    PRESSURE_STANDARD = 0x74,
    PRESSURE_HIGH_RESOLUTION = 0xB4,
    PRESSURE_ULTRA_HIGH_RESOLUTION = 0xF4,
} MeasureMode;

#define UT 27898
#define UP 23843
#define FIXED_BMP180_ID 0x55

static MeasureMode mode_selected = NONE_SELECTED;

void i2c_read(uint8_t address, uint8_t reg, uint8_t *data, size_t size) {

    if (address != SIMULATOR_ADDRESS) {
        printf("Wrong address...doing nothing\n");
    }

    uint32_t temporary = 0;
    if (size == 3) {
        switch (reg) {
            case OUT_MSB:
                printf("Accessed Register 3 byte OUT_MSB\n");
                if (mode_selected != TEMPERATURE)
                    temporary = UP;
                else {
                    printf("Register %d for Temperature not available "
                            "in 3 bytes", reg);
                    return;
                }
                break;
            default:
                printf("Register %d not available in 3 bytes", reg);
                return;
                break;
        }
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        temporary = ((temporary << 8) & 0xFF00FF00) | ((temporary >> 8) & 0xFF00FF);
        temporary = (temporary << 16) | ((temporary >> 16) & 0xFFFF);
        temporary >>= 16; //due to memcopy reading 2 leading 0s
#endif
    }
    else if (size == 2) {
        switch (reg) {
            case AC1:
                temporary = 408;
                printf("Accessed Register AC1  ");
                break;
            case AC2:
                temporary = -72;
                printf("Accessed Register AC2 ");
                break;
            case AC3:
                temporary = -14383;
                printf("Accessed Register AC3 ");
                break;
            case AC4:
                printf("Accessed Register AC4 ");
                temporary = 32741;
                break;
            case AC5:
                printf("Accessed Register AC5 ");
                temporary = 32757;
                break;
            case AC6:
                printf("Accessed Register AC6 ");
                temporary = 23153;
                break;
            case B1:
                printf("Accessed Register B1 ");
                temporary = 6190;
                break;
            case B2:
                printf("Accessed Register B2 ");
                temporary = 4;
                break;
            case MB:
                printf("Accessed Register MB ");
                temporary = -32758;
                break;
            case MC:
                printf("Accessed Register MC ");
                temporary = -8711;
                break;
            case MD:
                printf("Accessed Register MD ");
                temporary = 2868;
                break;
            case OUT_MSB:
                printf("Accessed Register OUT_MSB\n");
                if (mode_selected == TEMPERATURE)
                    temporary = UT;
                break;
            case CTRL_MEAS: //TODO
            default:
                printf("Wrong register for 2 bytes %x. Doing nothing\n", reg);
                return;
                break;
        }
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        temporary = ((temporary >> 8) & 0xFF) | (temporary << 8);
#endif
    }
    else if (size == 1) {
        switch (reg) {
            case CHIP_ID:
                temporary = FIXED_BMP180_ID;
                break;
            case OUT_MSB:
                printf("Accessed Register OUT_MSB\n");
                if (mode_selected == TEMPERATURE)
                    temporary = (UT & 0xFF00) >> 8;
                else
                    temporary = (UP & 0xFF00) >> 8;
                break;
            case OUT_LSB:
                printf("Accessed Register OUT_LSB\n");
                if (mode_selected == TEMPERATURE)
                    temporary = (UT & 0x00FF);
                else
                    temporary = (UP & 0x00FF);
                break;
            case OUT_XLSB:
                printf("Accessed Register OUT_XLSB\n");
                temporary = 0;
                break;
            default:
                printf("Wrong register %x. Doing nothing\n", reg);
                return;
        }
    }

    else
        printf("Wrong read size %lu\n", size);

    memcpy(data, &temporary, size);
    printf("\n");
}
// I2C burst write (writes to 7 but I2C device 'address' 'size' bytes,
//starting with register 'reg')

#define REGISTER_CTRL_MEAS 0xF4

void i2c_write(uint8_t address, uint8_t reg, uint8_t *data, size_t size) {
    if (address != SIMULATOR_ADDRESS) {
        return;
    }
    uint32_t temporary = 0;
    memcpy(&temporary, data, size);
    if (reg == REGISTER_CTRL_MEAS) {
        if (temporary >= TEMPERATURE &&
                temporary <= PRESSURE_ULTRA_HIGH_RESOLUTION)
            mode_selected = temporary;
    }
}