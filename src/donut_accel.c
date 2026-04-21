#include "donut_accel.h"

// Vector math stuff for the multi-acceleromter calcs
Vector2D vec_subtract(Vector2D v1, Vector2D v2) {
    Vector2D result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    return result;
}

float vec_magnitude(Vector2D v) {
    return sqrtf((v.x * v.x) + (v.y * v.y));
}

void accel_init(i2c_inst_t* i2c_port, uint8_t i2c_sda, uint8_t i2c_scl) {
    accelerometer_init(ACCEL_I2C_PORT, ACCEL_I2C_SDA, ACCEL_I2C_SCL);
}

// gets microseconds per rotation
// lies about upr depending on what right_x_percent is and what LEFT_RIGHT_HEADING_CONTROL_DIVISOR is to adjust direction
double get_rpm(double right_x_percent) {
    // fix this: get rpm from accel data along correct axis, WILL PROBABLY NEED TO CHANGE THE AXIS LATER
    double x_gs = accelerometer_get_x() - ACCEL_ZERO_G_OFFSET;

    // mapping [0,1] -> [-1,1]
    right_x_percent = (right_x_percent*2) - 1;

    double effective_radius_in_cm = ACCEL_MOUNT_RADIUS_CM + (ACCEL_MOUNT_RADIUS_CM * right_x_percent * LEFT_RIGHT_HEADING_CONTROL_DIVISOR);

    double rpm = fabs(x_gs - ACCEL_ZERO_G_OFFSET) * 89445.0f;
    rpm = rpm / effective_radius_in_cm;
    rpm = sqrt(rpm);

    return rpm;
}

// version for 2 accelerometers and doing magnitude from diagonals + distance
// Things needed:
// * framework for multiple accelerometers - needs to have offset for x/y, pin to read, etc.
// * fix the vector code (proper vector math)
// * way to store different 0 offeets from each accelerometer axis (4 total).

double get_rpm_2accel(double right_x_percent) {
    // X,Y,Z of all data
    double* Accel1RawData = Accel1.accelerometer_get_all_axis();
    double* Accel2RawData = Accel2.accelerometer_get_all_axis();

    // Convert to 2D vector for math stuff
    Vector2D Accel1Data, Accel2Data, Accel1Position, Accel2Position;

    Accel1Data.x = Accel1RawData[0];
    Accel1Data.y = Accel1RawData[1];

    Accel2Data.x = Accel2RawData[0];
    Accel2Data.y = Accel2RawData[1];

    // subtract the offsets 
    // Replace this with actual code when accelerometer struct/storage is set up.
    double xoffset = 0.0;
    double yoffset = 0.0;
    Vector2D Accel1Offsets = {xoffset, yoffset};
    Vector2D Accel2Offsets = {xoffset, yoffset};

    Accel1Data = vec_subtract(Accel1Data, Accel1Offsets);
    Accel2Data = vec_subtract(Accel2Data, Accel2Offsets);


    // Position vectors of the accelerometers (in cm)
    Accel1Position.x = 1.0 * 1.0/sqrtf(2.0);
    Accel1Position.y = 1.0 * 1.0/sqrtf(2.0);

    Accel2Position.x = -1.0 * 1.0/sqrtf(2.0);
    Accel2Position.y = -1.0 * 1.0/sqrtf(2.0);


    // omega = sqrt [ ||(a1 - a2)|| / ||(r2 - r1)|| ]

    Vector2D delta_A = vec_subtract(Accel1Data, Accel2Data);
    Vector2D delta_pos = vec_subtract(Accel2Position, Accel1Position);

    float mag_delta_A = vec_magnitude(delta_A);
    float mag_delta_Pos = vec_magnitude(delta_pos);
    
    // 89445f converts from Gs to RPM using gravity and angular acceleration.
    double rpm = sqrtf((mag_delta_A / mag_delta_Pos) * 89445.0f); // Gemini is telling me to make everything floats for optimization purposes

    return rpm;
}

double get_fake_rpm(double right_x_percent) {
    return FAKE_RPM;
}