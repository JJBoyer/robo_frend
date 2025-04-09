/*

Filename: encoder.cpp
Author: Jacob Boyer
Description:

*/

#include "esp_log.h"
#include "encoder.hpp"

volatile int64_t now = 0;
int64_t past = 0;
volatile bool meas_left_vel = false;
volatile bool meas_right_vel = false;

odometry_t wheel = {
    .time = 0,
    .left = 0.0f,
    .right = 0.0f,
    .avg = 0.0f,
    .w = 0.0f
};

void IRAM_ATTR left_isr_handler(void* arg){
    now = esp_timer_get_time();
    meas_left_vel = true;
};

void IRAM_ATTR right_isr_handler(void* arg){
    now = esp_timer_get_time();
    meas_right_vel = true;
};

void initEncoders(){

    // Config left encoder interrupt pin
    gpio_config_t leftA_conf = {
        .pin_bit_mask = (1ULL << LEFT_A),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&leftA_conf);

    // Config left encoder direction pin
    gpio_config_t leftB_conf = {
        .pin_bit_mask = (1ULL << LEFT_B),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&leftB_conf);

    // Config right encoder interrupt pin
    gpio_config_t rightA_conf = {
        .pin_bit_mask = (1ULL << RIGHT_A),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&rightA_conf);

    // Config right encoder direction pin
    gpio_config_t rightB_conf = {
        .pin_bit_mask = (1ULL << RIGHT_B),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&rightB_conf);

    // Install Left ISR service and attach interrupt
    gpio_install_isr_service(0);
    esp_err_t left_isr_error = gpio_isr_handler_add(LEFT_A, left_isr_handler, NULL);
    if(left_isr_error != ESP_OK){
        printf("Error: ISR Not Attached. Code: %d\n", left_isr_error);
        return;
    }

    // Report successful initialization
    status.set(ENCODERS);
}

void measVelocity(){

    // Calculate time step
    int dt = now - past;
    wheel.time = now;
    past = now;
    
    // Calculate wheel velocity
    float vWheel = 0.00883f / dt;

    // Lower flag and save velocity
    if(meas_left_vel){
        meas_left_vel = false;
        wheel.left = vWheel;
    } else if(meas_right_vel){
        meas_right_vel = false;
        wheel.right = vWheel;
    }

    // Average wheel velocities to find robot velocity
    wheel.avg = (wheel.right + wheel.left) / 2;

    // Calculate angular velocity (RH)
    wheel.w = (wheel.right - wheel.left) / wheelBaseWidth;

}