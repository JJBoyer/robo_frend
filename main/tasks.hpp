#ifndef TASKS_HPP
#define TASKS_HPP

#include "freertos/FreeRTOS.h"
#include "pwm_control.hpp"
#include "sensor_control.hpp"
#include "status.hpp"

// Declare function interfaces
void blinkLedTask(void* pvParameters);
void setLEDTask(void* pvParameters);
void ultrasonicTask(void* pvParameters);
void getStatusTask(void* pvParameters);
void initTasks();

#endif