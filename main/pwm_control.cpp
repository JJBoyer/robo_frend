/*

File Name: pwm_control.cpp
Author: Jacob Boyer
Description: Defines a function to initialize
the pwm configuration and channel, then defines
functions to use PWM to control LEDs and actuators

*/

#include "pwm_control.hpp"
#include "esp_log.h"

using namespace std;

/* initPWM:
  Performs the required configuration and initialization
  of the PWM channel for use by other functions. Currently
  sets a default 50% duty cycle upon initialization, but
  the default write can be removed.
*/
void initPWM(){  // Initialize the PWM timer and channel

    // Initialize PWM pin
    gpio_set_direction(PWM_PIN, GPIO_MODE_OUTPUT);

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

    // PWM Channel Config
    ledc_channel_config_t ledc_channel = {
        .gpio_num             = PWM_PIN,             // Set GPIO Pin
        .speed_mode           = LEDC_LOW_SPEED_MODE,
        .channel              = LEDC_CHANNEL_0,      // Channel 0 (up to 8 channels)
        .intr_type            = LEDC_INTR_DISABLE,   // Disable interrupts
        .timer_sel            = LEDC_TIMER_0,        // Use Timer 0
        .duty                 = 0,                   // Initial Duty Cycle (out of 1023 for 10-bit res)
        .hpoint               = 0,
        .flags                = {
            .output_invert    = 0                    // Required in ESP-IDF 5.0+
        }
    };
    
    esp_err_t channel_result = ledc_channel_config(&ledc_channel);
    if(channel_result != ESP_OK){
        printf("ERROR: LEDC Channel Config Failed! Error Code: %d", channel_result);
        return;
    } else {
        printf("LEDC Channel Configured Successfully!\n");
    }

    // Initialize the PWM pin with a default (50%) duty cycle
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

}

/* blink:
  Uses a PWM signal to adjust to blink an LED
  with a blink frequency and PWM duty cycle defined
  by the setFreq() and setBright() functions
*/
void blink(){  // Blink LED with PWM

    static int freqBackup = 500;  // Backup frequency if mutex not available
    static int dutyBackup = 512;  // Backup duty cycle if mutex not available

    // Request mutex and update duty cycle
    if(xSemaphoreTake(dutyMutex, portMAX_DELAY)){
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ((pduty) ? *pduty : 512));  // Turn LED on with duty cycle
        dutyBackup = *pduty;  // Save backup duty cycle
        xSemaphoreGive(dutyMutex);  // Return mutex
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyBackup);  // Use backup if mutex not available
    }

    // Apply the new duty cycle
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    // Request mutex and update frequency
    if(xSemaphoreTake(freqMutex, portMAX_DELAY)){
        vTaskDelay(pdMS_TO_TICKS((pfreq) ? *pfreq : 500)); // Adjust blink frequency
        freqBackup = *pfreq;  // Save backup frequency
        xSemaphoreGive(freqMutex);  // Return mutex
    } else {
        vTaskDelay(pdMS_TO_TICKS(freqBackup));  // Use backup if mutex not available
    }

    // Set LED pin low
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);  // Turn LED off
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);  // Actually apply the duty cycle
    
    // Request mutex and update frequency
    if(xSemaphoreTake(freqMutex, portMAX_DELAY)){
        vTaskDelay(pdMS_TO_TICKS((pfreq) ? *pfreq : 500)); // Adjust blink frequency
        freqBackup = *pfreq;  // Save backup frequency
        xSemaphoreGive(freqMutex);  // Return mutex
    } else {
        vTaskDelay(pdMS_TO_TICKS(freqBackup));  // Use backup if mutex not available
    }
}