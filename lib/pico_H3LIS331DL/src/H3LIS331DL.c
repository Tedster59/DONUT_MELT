#include "H3LIS331DL.h"

static accelerometer_t* accel_1;
static accelerometer_t* accel_2;

void write_to_register(accelerometer_t* accel, uint8_t register_address, uint8_t register_value){
    uint8_t buffer[2] = {register_address, register_value};
    i2c_write_blocking(accel->i2c_port, accel->accelerometer_address, buffer, 2, false);
}

void setup_accelerometer(accelerometer_t* accel) {
    accel->_curr_readings[0] = 0;
    accel->_curr_readings[1] = 0;
    accel->_curr_readings[2] = 0;

    i2c_init(accel->i2c_port, 400*1000);

    gpio_set_function(accel->i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(accel->i2c_scl, GPIO_FUNC_I2C);

    gpio_pull_up(accel->i2c_sda); //I2C pins need to be HIGH by default
    gpio_pull_up(accel->i2c_scl);

    write_to_register(accel, CTRL_REG1_REGISTER_ADDRESS, CTRL_REG1_REGISTER_VALUE); //Powers on accelerometer + sets output rate
    write_to_register(accel, CTRL_REG4_REGISTER_ADDRESS, CTRL_REG4_REGISTER_VALUE); //Updates block data update type, endianness, and accel scale 
}

void accelerometer_init(accelerometer_t* user_accel_1, accelerometer_t* user_accel_2){
    accel_1 = user_accel_1;
    accel_2 = user_accel_2;

    setup_accelerometer(accel_1);
    setup_accelerometer(accel_2);
}

// THIS FUNCTION IS PURELY FROM CLAUDE - Cai
bool is_data_ready_to_be_read(accelerometer_t* accel)
{
    uint8_t status_reg = 0x27;
    uint8_t status;
    i2c_write_blocking(accel->i2c_port, accel->accelerometer_address, &status_reg, 1, true);
    i2c_read_blocking(accel->i2c_port, accel->accelerometer_address, &status, 1, false);
    if (status & 0x08) { // ZYXDA bit - all axes have new data
    // read axis data
        return true;
    }
    return false;
}

double* accelerometer_get_all_axis(accelerometer_t* accel){ //Maybe make a struct to store/access accelerometer data more easily 
    if (!is_data_ready_to_be_read(accel)){
        return accel->_curr_readings;
    }

    uint8_t firstRegister = (1 << 7) | LSB_X_REGISTER_ADDRESS; //MSB is 1 for auto-increment during register reads
    i2c_write_blocking(accel->i2c_port, accel->accelerometer_address, &firstRegister, 1, true); //pretty sure this function handles setting the LSB of ACCELEROMETER_ADDRESS to 1 for write bc all I2C addresses are 7 bits 
    uint8_t buffer[6]; //need enough space for MSB register and LSB register of each axis
    i2c_read_blocking(accel->i2c_port, accel->accelerometer_address, buffer, 6, false); //same here with the read bit of the address, data stored in buffer 
    static double results[3];
    for (int i = 0; i < 3; ++i){
        // data is little endian (LSB register first). Combining the bytes explicitly avoids the old
        // uint8_t* -> uint16_t* cast (strict-aliasing/alignment UB in C); same result on this CPU.
        uint16_t word = (uint16_t)(buffer[2*i] | (buffer[2*i + 1] << 8));
        int16_t raw = (int16_t)word >> 4; // CLAUDE SAID TO DO THIS PART - Cai (12-bit left-justified value)
        results[i] = raw * ACCEL_SENSITIVITY / 1000.0; //divided by 1000.0 for gs + makes it a double
    }

    accel->_curr_readings[0] = results[0];
    accel->_curr_readings[1] = results[1];
    accel->_curr_readings[2] = results[2];

    return accel->_curr_readings;
}

double accelerometer_get_x(accelerometer_t* accel){
    accelerometer_get_all_axis(accel);
    return accel->_curr_readings[0];
}

double accelerometer_get_y(accelerometer_t* accel){
    accelerometer_get_all_axis(accel);
    return accel->_curr_readings[1];
}

double accelerometer_get_z(accelerometer_t* accel){
    accelerometer_get_all_axis(accel);
    return accel->_curr_readings[2];
}

// void pico_H3LIS331DL_is_library_accesible(){
//     printf("Accessing the pico_H3LIS331DL library is functioning properly! \n");
// }