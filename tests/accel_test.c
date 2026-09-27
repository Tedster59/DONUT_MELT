#include <stdio.h>
#include "H3LIS331DL.h"
#include "../src/donut_config.h"

#define RUNNING_A_TEST

static accelerometer_t accel_1 = {
    .accelerometer_address = ACCELEROMETER_ADDRESS,
    .i2c_port = ACCEL_1_I2C_PORT,
    .i2c_sda = ACCEL_1_I2C_SDA,
    .i2c_scl = ACCEL_1_I2C_SCL
};

static accelerometer_t accel_2 = {
    .accelerometer_address = ACCELEROMETER_ADDRESS,
    .i2c_port = ACCEL_2_I2C_PORT,
    .i2c_sda = ACCEL_2_I2C_SDA,
    .i2c_scl = ACCEL_2_I2C_SCL
};

int main() {
    stdio_init_all();

    accelerometer_init(&accel_1, &accel_2);
    sleep_ms(100); // give sensor time to boot I guess? - Cai

    while (1) {
        double* accel_1_data = accelerometer_get_all_axis(&accel_1);
        printf("accel_1 X axis: %lf \n", accel_1_data[0]);
        printf("accel_1 Y axis: %lf \n", accel_1_data[1]);
        printf("accel_1 Z axis: %lf \n\n", accel_1_data[2]);

        double* accel_2_data = accelerometer_get_all_axis(&accel_1);
        printf("accel_2 X axis: %lf \n", accel_2_data[0]);
        printf("accel_2 Y axis: %lf \n", accel_2_data[1]);
        printf("accel_2 Z axis: %lf \n\n", accel_2_data[2]);
    }
    
    return 0;
}