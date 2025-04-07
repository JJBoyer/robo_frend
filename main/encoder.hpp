/*



*/

#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "driver/gpio.h"
#include "globals.hpp"

typedef struct {
    int64_t time;
    float left;
    float right;
    float avg;
    float w;
} odometry_t;


void IRAM_ATTR left_isr_handler(void* arg);
void initEncoders();

#define LEFT_A GPIO_NUM_37
#define LEFT_B GPIO_NUM_38
#define RIGHT_A GPIO_NUM_35
#define RIGHT_B GPIO_NUM_36

#endif