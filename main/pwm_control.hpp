/*

File Name: pwm_control.hpp
Author: Jacob Boyer
Description: Header file for pwm_control.hpp

*/

#ifndef PWM_CONTROL_HPP
#define PWM_CONTROL_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"

// Declare extern global pointers for thread communication
extern std::unique_ptr<int> pfreq;
extern std::unique_ptr<int> pduty;

// Declare extern mutexes to unlock global pointers
extern SemaphoreHandle_t freqMutex;
extern SemaphoreHandle_t dutyMutex;

// Function to initialize motor control pins
void initMotors();

// Function to write PWM signal to LED
void forward();

// Define PWM hardware pin
#define LEFT_PWM_PIN GPIO_NUM_2    // Connects to ENA on driver
#define RIGHT_PWM_PIN GPIO_NUM_42  // Connects to ENB on driver

// Define motor direction hardware pins
#define LEFT_FORWARD GPIO_NUM_7   // Connects to IN1 on driver
#define LEFT_REVERSE GPIO_NUM_6   // Connects to IN2 on driver
#define RIGHT_REVERSE GPIO_NUM_5  // Connects to IN3 on driver
#define RIGHT_FORWARD GPIO_NUM_4  // Connects to IN4 on driver

#endif