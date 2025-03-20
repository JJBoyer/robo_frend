#ifndef PWM_CONTROL_HPP
#define PWM_CONTROL_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"

extern SemaphoreHandle_t freqMutex;
extern SemaphoreHandle_t dutyMutex;

extern std::unique_ptr<int> pfreq;
extern std::unique_ptr<int> pduty;

void blink();
void initPWM();

#define PWM_PIN GPIO_NUM_2  // Pin for PWM output

#endif