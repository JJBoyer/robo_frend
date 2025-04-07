/*

Filename: imu.cpp
Author: Jacob Boyer
Description: Defines functions to initialize
the MPU6050 IMU and parse the data from the IMU
to a more usable format for the control and 
navigation tasks.

*/

#include <cmath>
#include "imu.hpp"
#include "esp_log.h"
#include "mutex_guard.hpp"

using namespace std;

#define MPU6050_CONFIG_REGISTER 0x1A
#define MPU6050_DLPF_MODE 0x03
#define MPU_ADDRESS 0x68

// Create IMU handle
mpu6050_handle_t mpu_sensor = NULL;

// Create IMU data variables
mpu6050_acce_value_t accel;
mpu6050_acce_value_t accelBias;
float accelGain;
mpu6050_gyro_value_t gyro;
mpu6050_gyro_value_t gyroBias;

/* init6050:
    Initializes and configures the MPU6050
    IMU, then samples data in order to calibrate
    and prepare the sensor for navigation tasks.
*/
void init6050(){

    // Initialize the IMU handle
    mpu_sensor = mpu6050_create(I2C_NUM_0, 0x68);
    if(mpu_sensor == NULL){
        printf("MPU6050 handle is NULL!");
        return;
    }

    // Confirm connection to the IMU over the I2C bus
    scanBus(I2C_NUM_0);
    wakeIMU();
    vTaskDelay(pdMS_TO_TICKS(100));
    setDLPF();

    // Measure bias on startup and test calibration on 
    // an initial measurement
    getGyroBias();
    getAccelBias();
    measureAccel(mpu_sensor);
    measureGyro(mpu_sensor);

    // Display test values
    printf("Accel Data: x = %f, y = %f, z = %f\n", accel.acce_x, accel.acce_y, accel.acce_z);
    printf("Gyro Data: x = %f, y = %f, z = %f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
    
}

/* scanBus:
    This utility function allows the ESP32
    to search for any devices on the I2C line
    to verify connection to said devices. The
    function displays the address of each device
    found.
*/
void scanBus(i2c_port_t port) {

    printf("I2C scanning...\n");

    // Ping every address for a response
    for (int addr = 1; addr < 127; addr++) {
        // Generate a write command for each address
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        // Send the command and wait 100ms for an ACK
        esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        // If an address responds, display the address on the serial monitor
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

    // Define wake_cmd byte
    uint8_t wake_cmd[2] = {0x6B, 0x00};  // PWR_MGMT_1 = 0 (wake up)

    // Generate the command to wake the IMU
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x68 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, wake_cmd, 2, true);
    i2c_master_stop(cmd);

    // Send command and await an ACK to confirm reception
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        printf("Failed to wake MPU6050: %s\n", esp_err_to_name(err));
    } else {
        printf("IMU was awoken!\n");
    }

}

/* setDLPF:
    Sets the digital low pass filter register to
    control the MPU6050's internal filtering to
    reduce noise.
*/
void setDLPF(){

    uint8_t dlpf_config[2] = {MPU6050_CONFIG_REGISTER, MPU6050_DLPF_MODE};  // Config register and value

    // Create command to set DLPF register
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, dlpf_config, 2, true);
    i2c_master_stop(cmd);

    // Send command
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

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
    gyroBias.gyro_x = gyro_sum.gyro_x / samples;
    gyroBias.gyro_y = gyro_sum.gyro_y / samples;
    gyroBias.gyro_z = gyro_sum.gyro_z / samples;

}

/* getAccelBias:
    Takes 200 acceleration samples and finds the
    magnitude of the total acceleration for each
    sample. The average of the samples is computed
    and saved as the gain of the readings.
    The bias is also found after applying the gain.
*/
void getAccelBias(){

    float accel_gain_sum = 0.0f;
    mpu6050_acce_value_t accel_bias_sum = {0};
    const int samples = 200;

    // Collect samples to measure gain
    for (int i = 0; i < samples; i++){
        
        mpu6050_acce_value_t a;
        mpu6050_get_acce(mpu_sensor, &a);

        accel_gain_sum += sqrtf(a.acce_x*a.acce_x + a.acce_y*a.acce_y + a.acce_z*a.acce_z);

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    accelGain = accel_gain_sum / samples;

    for (int i = 0; i < samples; i++){

        mpu6050_acce_value_t a;
        mpu6050_get_acce(mpu_sensor, &a);

        accel_bias_sum.acce_x += a.acce_x / accelGain;
        accel_bias_sum.acce_y += a.acce_y / accelGain;
        accel_bias_sum.acce_z += (a.acce_z / accelGain) - 1;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    accelBias.acce_x = accel_bias_sum.acce_x / samples;
    accelBias.acce_y = accel_bias_sum.acce_y / samples;
    accelBias.acce_z = accel_bias_sum.acce_z / samples;

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

    // Remove gain from the values
    accel.acce_x /= accelGain;
    accel.acce_y /= accelGain;
    accel.acce_z /= accelGain;

    // Remove bias from the values
    accel.acce_x -= accelBias.acce_x;
    accel.acce_y -= accelBias.acce_y;
    accel.acce_z -= accelBias.acce_z;

}

/* measureGyro:
    Collects gyroscope data from the MPU6050
    and updates the gyro data variable after
    applying the necessary calibration for
    more accurate data. Provides measurements
    in rad/s.
*/
void measureGyro(mpu6050_handle_t& mpu_sensor){

    // Collect gyro data from IMU
    mpu6050_get_gyro(mpu_sensor, &gyro);

    // Apply bias negation
    gyro.gyro_x -= gyroBias.gyro_x;
    gyro.gyro_y -= gyroBias.gyro_y;
    gyro.gyro_z -= gyroBias.gyro_z;

    // Convert default degrees to radians for consistent computation
    gyro.gyro_x *= deg_to_rad;
    gyro.gyro_y *= deg_to_rad;
    gyro.gyro_z *= deg_to_rad;

}

/* clampToZero:
    A special clamp function that clamps a
    known range of uncertainty in a sensor
    to zero.
*/
float clampToZero(float value, float threshold){
    return (fabs(value) < threshold) ? 0.0f : value;
}

/* estimateState:
    Collects accelerometer data from IMU and
    uses trapezoidal integration to estimate the
    position and velocity components of the system
    state. Exports the estimated state to pstate.
*/
void estimateState(mpu6050_handle_t& mpu_sensor){

    // Initialize state_t for current state
    static state_t this_state = {
        .posX = 0,
        .posY = 0,
        .th   = 0,
        .velX = 0,
        .velY = 0,
        .w    = 0,
        .acc  = 0
    };

    // Initialize state_t for previous state
    static state_t last_state = {
        .posX = 0,
        .posY = 0,
        .th   = 0,
        .velX = 0,
        .velY = 0,
        .w    = 0,
        .acc  = 0
    };

    // Calculate dt for this loop
    static int64_t last_time = 0;
    int64_t now = esp_timer_get_time();  // microseconds
    float dt = (now - last_time) / 1e6;  // seconds
    last_time = now;

    // Collect data for this timestep
    measureAccel(mpu_sensor);
    measureGyro(mpu_sensor);

    // Compute angular state estimation
    this_state.w = clampToZero(gyro.gyro_z, 0.15f);
    this_state.th = fmod(last_state.th + (last_state.w + this_state.w) * dt / 2, 2*M_PI);
    if(this_state.th < 0) this_state.th += 2*M_PI; // Maintain positive heading value

    // Compute linear state estimation in 2D
    this_state.acc = clampToZero((9.81 * accel.acce_y), 0.05f);  // robot y-axis is forward
    this_state.velX = last_state.velX + (last_state.acc * sin(last_state.th) + this_state.acc * sin(this_state.th)) * dt / 2;
    this_state.velY = clampToZero((last_state.velY + (last_state.acc * cos(last_state.th) + this_state.acc * cos(this_state.th)) * dt / 2), 0.005f);
    this_state.posX = last_state.posX + (last_state.velX + this_state.velX) * dt / 2;
    this_state.posY = last_state.posY + (last_state.velY + this_state.velY) * dt / 2;

    // Save state for this time step for use in next time step
    last_state = this_state;

    // Write new state to global pointer for use by GNC tasks
    {
        MutexGuard lock(stateMutex);
        *pstate = this_state;    
    }

    static int log_counter = 0;
    // Display test values
    //printf("\033[2J\033[H");
    if(++log_counter % 40 == 0){ // Print about once per second
        printf("Accel Data: x = %.2f, y = %.2f, z = %.2f\n", accel.acce_x, accel.acce_y, accel.acce_z);
        printf("Gyro Data: x = %.2f, y = %.2f, z = %.2f\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        printf("[x, y, th, vx, vy, w, a, dt]\n");
        printf("[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n", this_state.posX, this_state.posY, this_state.th, this_state.velX, this_state.velY, this_state.w, this_state.acc, dt);
    }
}