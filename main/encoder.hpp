/*



*/

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "driver/gpio.h"
#include "esp_timer.h"
#include "globals.hpp"

// Define a new odometry struct for measuring encoder values
typedef struct {
    int64_t time;
    float left;
    float right;
    float avg;
    float w;
} odometry_t;

// Declare wheel odometry struct for use in state estimation
extern odometry_t wheel;

// ISR handler for 
void IRAM_ATTR left_isr_handler(void* arg);
void IRAM_ATTR right_isr_handler(void* arg);
void initEncoders();
void measureVelocity();

#define LEFT_A GPIO_NUM_37
#define LEFT_B GPIO_NUM_38
#define RIGHT_A GPIO_NUM_35
#define RIGHT_B GPIO_NUM_36

#endif