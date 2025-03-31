/*

Filename: sensor_control.hpp
Author: Jacob Boyer
Description: Header file for sensor_control.cpp

*/

#ifndef SENSOR_CONTROL_HPP
#define SENSOR_CONTROL_HPP

#include "driver/gpio.h"
#include "esp_timer.h"
#include "globals.hpp"

// Declare extern int64s for ultrasonic distance measurement
extern volatile int64_t start_time;
extern volatile int64_t end_time;

// Function to initialize all sensors
void initSensors();

// Function to initialize the HC-SR04 ultrasonic sensor
void initSR04();

// Function to initialize the I2C bus
void initI2C();

// Function to measure ultrasonic distance
void getDistance();

// ADC Related Pins
#define READ_PIN ADC_CHANNEL_0  // Example: GPIO1 (ADC1_CH0), set the GPIO1 pin to analog input

// I2C Pins
#define SDA GPIO_NUM_11         // I2C SDA Pin (Default 21)
#define SCL GPIO_NUM_10         // I2C SCL Pin (Default 22)

// HC-SR04 Pins
#define TRIG GPIO_NUM_39        // Ultrasonic TRIG Pin
#define ECHO GPIO_NUM_40        // Ultrasonic ECHO Pin

#endif