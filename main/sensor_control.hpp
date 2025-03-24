/*

File Name: sensor_control.hpp
Author: Jacob Boyer
Description: Header file for sensor_control.cpp

*/

#ifndef SENSOR_CONTROL_HPP
#define SENSOR_CONTROL_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// Declare extern global pointers for thread communication
extern std::unique_ptr<int> pfreq;
extern std::unique_ptr<int> pduty;
extern std::unique_ptr<double> pdist;

// Declare extern mutexes for unlocking global pointers
extern SemaphoreHandle_t freqMutex;
extern SemaphoreHandle_t dutyMutex;
extern SemaphoreHandle_t distMutex;

// Declare extern int64s for ultrasonic distance measurement
extern volatile int64_t start_time;
extern volatile int64_t end_time;

// Function to initialize all sensors
void initSensors();

// Function to initialize the ADC
void initADC();

// Function to initialize the HC-SR04 ultrasonic sensor
void initSR04();

// Function to initialize the I2C bus
void initI2C();

// Function to set PWM duty cycle
void setBright();

// Function to set blink frequency
void setFreq();

// Function to measure ultrasonic distance
void getDistance();

// Function to collect data from a potentiometer
int readPot();

// ADC Related Pins
#define READ_PIN ADC_CHANNEL_0  // Example: GPIO1 (ADC1_CH0), set the GPIO1 pin to analog input

// I2C Pins
#define SDA GPIO_NUM_48         // I2C SDA Pin (Default 21)
#define SCL GPIO_NUM_15         // I2C SCL Pin (Default 22)

// HC-SR04 Pins
#define TRIG GPIO_NUM_39        // Ultrasonic TRIG Pin
#define ECHO GPIO_NUM_40        // Ultrasonic ECHO Pin
#define CHECK GPIO_NUM_47       // Ultrasonic Test Pin

#endif