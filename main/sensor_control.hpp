#ifndef SENSOR_CONTROL_HPP
#define SENSOR_CONTROL_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"

extern SemaphoreHandle_t freqMutex;
extern SemaphoreHandle_t dutyMutex;
extern SemaphoreHandle_t distMutex;

extern std::unique_ptr<int> pfreq;
extern std::unique_ptr<int> pduty;
extern std::unique_ptr<double> pdist;

extern volatile int64_t start_time;
extern volatile int64_t end_time;

void initSensors();
void setBright();
void setFreq();
void getDistance();
int readPot();

// ADC Related Pins
#define READ_PIN ADC_CHANNEL_0  // Example: GPIO1 (ADC1_CH0), set the GPIO1 pin to analog input

// I2C Pins
#define SDA GPIO_NUM_48         // I2C SDA Pin (Default 21)
#define SCL GPIO_NUM_15         // I2C SCL Pin (Default 22)

// HC-SR04 Pins
#define TRIG GPIO_NUM_39        // Ultrasonic TRIG Pin
#define ECHO GPIO_NUM_40        // Ultrasonic ECHO Pin

#endif