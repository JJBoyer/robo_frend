#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Declare a state_t struct for use in state estimation
typedef struct {
    float pos;
    float vel;
    float acc;
} state_t;

// Declare external global pointers for thread communication
extern std::unique_ptr<int> pduty;
extern std::unique_ptr<state_t> pstate;
extern std::unique_ptr<double> pdist;

// Declare external mutexes to access global pointers
extern SemaphoreHandle_t dutyMutex;
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t distMutex;

void initGlobals();

#endif