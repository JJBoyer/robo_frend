/*

Filename: task_manager.hpp
Author: Jacob Boyer
Description: Header file for task_manager.cpp

*/

#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks.hpp"
#include "imu.hpp"
#include "globals.hpp"

// Declare task handles for task management
extern TaskHandle_t motorHandle;
extern TaskHandle_t sonicHandle;
extern TaskHandle_t encoderHandle;
extern TaskHandle_t estimateHandle;
extern TaskHandle_t pursuitHandle;
extern TaskHandle_t teleHandle;

// Task initialization function
void initTasks();

// Task monitoring function
void printTaskStatus();

#endif