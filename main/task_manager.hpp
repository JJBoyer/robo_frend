#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "tasks.hpp"

// Declare pointers for inter-thread communication
extern std::unique_ptr<int> pfreq;
extern std::unique_ptr<int> pduty;
extern std::unique_ptr<double> pdist;

// Declare semaphores for memory locking
extern SemaphoreHandle_t freqMutex;
extern SemaphoreHandle_t dutyMutex;
extern SemaphoreHandle_t distMutex;

// Declare task handles for task management
extern TaskHandle_t setHandle;
extern TaskHandle_t blinkHandle;
extern TaskHandle_t sonicHandle;
extern TaskHandle_t statHandle;

// Task initialization function
void initTasks();

// Task monitoring function
void printTaskStatus();

#endif