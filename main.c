#include <stdio.h>
#include "bmp180Simulator.h"
#include "bmp180Driver.h"

int main() {

    BMP180Data bmp180 = {SIMULATOR_ADDRESS,
        {}, TEMPERATURE};

    double temperature = readTemperature(&bmp180);

    bmp180.measure_mode = PRESSURE_ULTRA_LOW_POWER;
    double pressure = readPressure(&bmp180);
    printf("Temp %f Press %f\n", temperature, pressure);
    return 0;
}