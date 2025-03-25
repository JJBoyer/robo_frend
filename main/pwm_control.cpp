/*

File Name: pwm_control.cpp
Author: Jacob Boyer
Description: Defines a function to initialize
the pwm configuration and channel, then defines
functions to use PWM to control LEDs and actuators

*/

#include "pwm_control.hpp"
#include "esp_log.h"
#include "mutex_guard.hpp"

#define LEFT_MOTOR LEDC_CHANNEL_0
#define RIGHT_MOTOR LEDC_CHANNEL_1

using namespace std;

/* initPWM:
  Performs the required configuration and initialization
  of the PWM channel for use by other functions. Currently
  sets a default 50% duty cycle upon initialization, but
  the default write can be removed.

  NOTE: Right PWM currently deactivated!
*/
void initMotors(){  // Initialize the motor control pins

    // Left motor PWM pin (ENA)
    gpio_config_t LPWM_conf = {
        .pin_bit_mask = (1ULL << LEFT_PWM_PIN), // Set pin assignment
        .mode = GPIO_MODE_OUTPUT                // Set to output
    };
    gpio_config(&LPWM_conf);

    // Right motor PWM pin (ENB)
    gpio_config_t RPWM_conf = {
        .pin_bit_mask = (1ULL << RIGHT_PWM_PIN), // Set pin assignment
        .mode = GPIO_MODE_OUTPUT                 // Set to output
    };
    gpio_config(&RPWM_conf);

    // Left motor direction pin 1 (IN1)
    gpio_config_t LDIR1_conf = {
        .pin_bit_mask = (1ULL << LEFT_DIR1_PIN), // Set pin assignment
        .mode = GPIO_MODE_OUTPUT                 // Set to output
    };
    gpio_config(&LDIR1_conf);

    // Left motor direction pin 2 (IN2)
    gpio_config_t LDIR2_conf = {
        .pin_bit_mask = (1ULL << LEFT_DIR2_PIN), // Set pin assignment
        .mode = GPIO_MODE_OUTPUT                 // Set to output
    };
    gpio_config(&LDIR2_conf);

    // Right motor direction pin 1 (IN3)
    gpio_config_t RDIR1_conf = {
        .pin_bit_mask = (1ULL << RIGHT_DIR1_PIN), // Set pin assignment
        .mode = GPIO_MODE_OUTPUT                  // Set to output
    };
    gpio_config(&RDIR1_conf);

    // Right motor direction pin 2 (IN4)
    gpio_config_t RDIR2_conf = {
        .pin_bit_mask = (1ULL << RIGHT_DIR2_PIN), // Set pin assignment
        .mode = GPIO_MODE_OUTPUT                  // Set to output
    };
    gpio_config(&RDIR2_conf);

    // PWM Timer Config
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,  // Low-speed mode for most applications
        .duty_resolution  = LEDC_TIMER_10_BIT,    // 10-bit resolution (0-1023)
        .timer_num        = LEDC_TIMER_0,         // Timer 0 (up to 4 timers available)
        .freq_hz          = 5000,                 // Frequency: 5 kHz
        .clk_cfg          = LEDC_USE_APB_CLK      // Use the best available clock
    };
    
    esp_err_t timer_result = ledc_timer_config(&ledc_timer);
    if(timer_result != ESP_OK){
        printf("ERROR: LEDC Timer Config Failed! Error Code: %d", timer_result);
        return;
    } else {
        printf("LEDC Timer Configured Successfully!\n");
    }

    // PWM Left Channel Config
    ledc_channel_config_t ledc_left_channel = {
        .gpio_num             = LEFT_PWM_PIN,        // Set GPIO Pin
        .speed_mode           = LEDC_LOW_SPEED_MODE,
        .channel              = LEFT_MOTOR,          // Channel 0 (up to 8 channels)
        .intr_type            = LEDC_INTR_DISABLE,   // Disable interrupts
        .timer_sel            = LEDC_TIMER_0,        // Use Timer 0
        .duty                 = 0,                   // Initial Duty Cycle (out of 1023 for 10-bit res)
        .hpoint               = 0,
        .flags                = {
            .output_invert    = 0                    // Required in ESP-IDF 5.0+
        }
    };
    
    esp_err_t left_channel_result = ledc_channel_config(&ledc_left_channel);
    if(left_channel_result != ESP_OK){
        printf("ERROR: LEDC Left Channel Config Failed! Error Code: %d", left_channel_result);
        return;
    } else {
        printf("LEDC Left Channel Configured Successfully!\n");
    }

    // PWM Right Channel Config
    ledc_channel_config_t ledc_right_channel = {
        .gpio_num             = RIGHT_PWM_PIN,       // Set GPIO Pin
        .speed_mode           = LEDC_LOW_SPEED_MODE,
        .channel              = RIGHT_MOTOR,         // Channel 1 (up to 8 channels)
        .intr_type            = LEDC_INTR_DISABLE,   // Disable interrupts
        .timer_sel            = LEDC_TIMER_0,        // Use Timer 0
        .duty                 = 0,                   // Initial Duty Cycle (out of 1023 for 10-bit res)
        .hpoint               = 0,
        .flags                = {
            .output_invert    = 0                    // Required in ESP-IDF 5.0+
        }
    };
    
    esp_err_t right_channel_result = ledc_channel_config(&ledc_right_channel);
    if(right_channel_result != ESP_OK){
        printf("ERROR: LEDC Right Channel Config Failed! Error Code: %d", right_channel_result);
        return;
    } else {
        printf("LEDC Right Channel Configured Successfully!\n");
    }

    // Initialize both PWM pins with a default (50%) duty cycle
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR, 512);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, RIGHT_MOTOR, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, RIGHT_MOTOR);

}

/* blink:
  Uses a PWM signal to adjust to blink an LED
  with a blink frequency and PWM duty cycle defined
  by the setFreq() and setBright() functions
*/
void blink(){  // Note: Repurpose to motor control

    // Request mutex and update duty cycle
    {
        MutexGuard lock(dutyMutex);
        // Turn LED on with duty cycle
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR, ((pduty) ? *pduty : 512));
    }

    // Apply the new duty cycle
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR);

    // Request mutex and update on length
    {
        MutexGuard lock(freqMutex);
        vTaskDelay(pdMS_TO_TICKS((pfreq) ? *pfreq : 500)); // Adjust blink frequency
    }

    // Set LED pin low
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR, 0);  // Turn LED off
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEFT_MOTOR);  // Actually apply the duty cycle
    
    // Request mutex and update off length
    {
        MutexGuard lock(freqMutex);
        vTaskDelay(pdMS_TO_TICKS((pfreq) ? *pfreq : 500)); // Adjust blink frequency
    }
}