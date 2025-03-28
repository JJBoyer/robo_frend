/*

    Filename: imu.cpp
    Author: Jacob Boyer
    Description: Defines functions to initialize
    the MPU6050 IMU and parse the data from the IMU
    to a more usable format for the control and 
    navigation tasks.

*/

#include "imu.hpp"
#include "esp_log.h"
#include "mutex_guard.hpp"

using namespace std;

// Create IMU handle
mpu6050_handle_t mpu_sensor = NULL;

// Create IMU data holders
mpu6050_acce_value_t accel;
mpu6050_gyro_value_t gyro;
mpu6050_gyro_value_t gyro_bias;

/* init6050:
    Initializes and configures the MPU6050
    IMU, then samples data in order to calibrate
    and prepare the sensor for navigation tasks.
*/
void init6050(){

    mpu_sensor = mpu6050_create(I2C_NUM_0, 0x68);

    if(mpu_sensor == NULL){
        printf("MPU6050 handle is NULL!");
        return;
    }

    scan_i2c_bus(I2C_NUM_0);
    wakeIMU();
    vTaskDelay(pdMS_TO_TICKS(100));
    getGyroBias();
    measureAccel(mpu_sensor);
    measureGyro(mpu_sensor);

    printf("Accel Data: x = %.2f, y = %.2f, z = %.2f\n", accel.acce_x, accel.acce_y, accel.acce_z);
    printf("Gyro Data: x = %.2f, y = %.2f, z = %.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
    
}

/* scan_i2c_bus:
    This utility function allows the ESP32
    to search for any devices on the I2C line
    to verify connection to said devices. The
    function displays the address of each device
    found.
*/
void scan_i2c_bus(i2c_port_t port) {
    printf("I2C scanning...\n");
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        if (err == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
        }
    }
    printf("Scan complete.\n");
}

/* wakeIMU:
    The MPU6050 defaults to sleep mode
    upon startup, so this command is given
    immediately after powering on in order
    to activate the IMU for data collection.
*/
void wakeIMU(){

    printf("Waking IMU...\n");
    uint8_t wake_cmd[2] = {0x6B, 0x00};  // PWR_MGMT_1 = 0 (wake up)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, wake_cmd, 2, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        printf("Failed to wake MPU6050: %s\n", esp_err_to_name(err));
    } else {
        printf("IMU was awoken!\n");
    }

}

/* getGyroBias:
    Takes 200 samples while the robot is
    stationary to measure bias in the gyro
    readings. These values are computed and
    then saved for future use in calibration.
*/
void getGyroBias(){

    mpu6050_gyro_value_t gyro_sum = {0};
    const int samples = 200;

    // Collect samples to measure bias
    for (int i = 0; i < samples; i++){
        
        mpu6050_gyro_value_t g;
        mpu6050_get_gyro(mpu_sensor, &g);

        gyro_sum.gyro_x += g.gyro_x;
        gyro_sum.gyro_y += g.gyro_y;
        gyro_sum.gyro_z += g.gyro_z;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Update gyro_bias for use in calculations
    gyro_bias.gyro_x = gyro_sum.gyro_x / samples;
    gyro_bias.gyro_y = gyro_sum.gyro_y / samples;
    gyro_bias.gyro_z = gyro_sum.gyro_z / samples;

}

/* measureAccel:
    Collects accelerometer data from the
    MPU6050 and saves it to the accel data
    variable. Calibration may be added if
    needed.
*/
void measureAccel(mpu6050_handle_t& mpu_sensor){

    // Collect accel data from IMU
    mpu6050_get_acce(mpu_sensor, &accel);

    // Add calibration here if needed

}

/* measureGyro:
    Collects gyroscope data from the MPU6050
    and updates the gyro data variable after
    applying the necessary calibration for
    more accurate data.
*/
void measureGyro(mpu6050_handle_t& mpu_sensor){

    // Collect gyro data from IMU
    mpu6050_get_gyro(mpu_sensor, &gyro);

    // Apply bias negation
    gyro.gyro_x -= gyro_bias.gyro_x;
    gyro.gyro_y -= gyro_bias.gyro_y;
    gyro.gyro_z -= gyro_bias.gyro_z;

}

/* estimateState:
    Collects accelerometer data from IMU and
    uses trapezoidal integration to estimate the
    position and velocity components of the system
    state. Exports the estimated state to pstate.
*/
void estimateState(mpu6050_handle_t& mpu_sensor){

    static state_t this_state = {
        .pos = 0,
        .vel = 0,
        .acc = 0
    };

    static state_t last_state = {
        .pos = 0,
        .vel = 0,
        .acc = 0
    };

    const float dt = 0.025;  // seconds

    // Collect data for this timestep
    measureAccel(mpu_sensor);

    // Compute state estimation
    this_state.acc = last_state.acc + 9.81 * accel.acce_y;  // Y-axis is forward
    this_state.vel = last_state.vel + (last_state.acc + this_state.acc) * dt / 2;
    this_state.pos = last_state.pos + (last_state.vel + this_state.vel) * dt / 2;

    // Save state for this time step for use in next time step
    last_state = this_state;

    // Write new state to global pointer for use by GNC tasks
    {
        MutexGuard lock(stateMutex);
        pstate.reset(new state_t(this_state));    
    }

}