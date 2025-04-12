/*



*/

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "driver/gpio.h"
#include "esp_timer.h"
#include "globals.hpp"

// Declare wheel odometry struct for use in state estimation
extern odometry_t wheel;

// Declare external variables for timing and encoder interrupts
extern volatile int64_t now;
extern volatile int64_t past;
extern volatile bool meas_left_vel;
extern volatile bool meas_right_vel;

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