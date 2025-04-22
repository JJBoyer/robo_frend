/*

Filename: tasks.hpp
Author: Jacob Boyer
Description: Header file for tasks.cpp

*/

#ifndef TASKS_HPP
#define TASKS_HPP

#include "freertos/FreeRTOS.h"
#include "motor_control.hpp"
#include "sensor_control.hpp"
#include "telemetry.hpp"
#include "state_estimation.hpp"
#include "pure_pursuit.hpp"

/*
Declare functions for tasks designed to run
the various robot operations at fixed intervals
*/
void motorTask(void* pvParameters);
void ultrasonicTask(void* pvParameters);
void encoderTask(void* pvParameters);
void estimateStateTask(void* pvParameters);
void purePursuitControlTask(void* pvParameters);
void sendTelemetryTask(void* pvParameters);

#endif