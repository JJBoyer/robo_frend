/*

Filename: imu.hpp
Author: Jacob Boyer
Description: Header file for imu.cpp

*/

#ifndef IMU_HPP
#define IMU_HPP

#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_timer.h"
#include "globals.hpp"

extern mpu6050_handle_t mpu_sensor;

// Function to initialize the MPU6050 IMU
void init6050();

// Function to scan for I2C devices
void scanBus(i2c_port_t port);

// Function to activate IMY on startup
void wakeIMU();

// Function to set digital low pass filter register
void setDLPF();

// Function to test for gyro bias
void getGyroBias();

// Function to test for accelerometer bias
void getAccelBias();

// Function to measure and calibrate acceleration data
void measureAccel(mpu6050_handle_t& mpu_sensor);

// Function to measure and calibrate gyro data
void measureGyro(mpu6050_handle_t& mpu_sensor);

// Function to prevent sensor noise from accumulating as drift
float clampToZero(float value, float threshold);

// Function to take the acclerometer data and
// estimate the system state.
void estimateState(mpu6050_handle_t& mpu_sensor);

#endif