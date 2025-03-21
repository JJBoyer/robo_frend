/*

File Name: tasks.hpp
Author: Jacob Boyer
Description: Header file for tasks.cpp

*/

#ifndef TASKS_HPP
#define TASKS_HPP

#include "freertos/FreeRTOS.h"
#include "pwm_control.hpp"
#include "sensor_control.hpp"
#include "status.hpp"

/*
Declare functions for tasks designed to run
the various robot operations at fixed intervals
*/
void blinkLedTask(void* pvParameters);
void setLEDTask(void* pvParameters);
void ultrasonicTask(void* pvParameters);
void getStatusTask(void* pvParameters);
void initTasks();

#endif