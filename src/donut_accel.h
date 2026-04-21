#ifndef DONUT_ACCEL_H
#define DONUT_ACCEL_H

#include "donut_config.h"

// A basic 2D Vector structure for the accelerometer data
typedef struct {
    float x;
    float y;
} Vector2D;

Vector2D vec_subtract(Vector2D v1, Vector2D v2);
float vec_magnitude(Vector2D v);

void accel_init(i2c_inst_t* i2c_port, uint8_t i2c_sda, uint8_t i2c_scl);

double get_rpm(double right_x_percent);

double get_rpm_2accel(double right_x_percent);

double get_fake_rpm(double right_x_percent);

#endif