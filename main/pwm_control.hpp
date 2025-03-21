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

// Function to initialize PWM configuration and channel
void initPWM();

// Function to write PWM signal to LED
void blink();

// Define PWM hardware pin
#define PWM_PIN GPIO_NUM_2

#endif