#ifndef STATUS_HPP
#define STATUS_HPP

#include <memory>  // allows use of memory tools such as unique_ptr
#include "freertos/FreeRTOS.h"  // allows use of FreeRTOS functions
#include "freertos/task.h"  // allows use of task-related functions
#include "freertos/semphr.h"  // allows use of semaphores

extern std::unique_ptr<int> pduty;
extern std::unique_ptr<int> pfreq;
extern std::unique_ptr<double> pdist;

extern SemaphoreHandle_t dutyMutex;
extern SemaphoreHandle_t freqMutex;
extern SemaphoreHandle_t distMutex;

void getStatus();

#endif