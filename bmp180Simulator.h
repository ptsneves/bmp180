#ifndef BMP180SIMULATOR_H
#define BMP180SIMULATOR_H

#include <stdint.h>
#include <stddef.h>
// I2C burst read (reads from 7 bit I2C device 'address' 'size' bytes
//starting with register 'reg' into 'data')

#define SIMULATOR_ADDRESS 0x10

void i2c_read(uint8_t address, uint8_t reg, uint8_t *data, size_t size);
// I2C burst write (writes to 7 but I2C device 'address' 'size' bytes,
//starting with register 'reg')

void i2c_write(uint8_t address, uint8_t reg, uint8_t *data, size_t size);
#endif
