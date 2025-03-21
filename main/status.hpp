/*

File Name: status.hpp
Author: Jacob Boyer
Description: Header file for status.hpp

*/

#ifndef STATUS_HPP
#define STATUS_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Declare external global pointers for thread communication
extern std::unique_ptr<int> pduty;
extern std::unique_ptr<int> pfreq;
extern std::unique_ptr<double> pdist;

// Declare external mutexes to access global pointers
extern SemaphoreHandle_t dutyMutex;
extern SemaphoreHandle_t freqMutex;
extern SemaphoreHandle_t distMutex;

// Function to display desired debugging data
void getStatus();

#endif