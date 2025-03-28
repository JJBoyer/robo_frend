#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Declare a state_t struct for use in state estimation
typedef struct {
    float posX; // position along global x-axis
    float posY; // position along global y-axis
    float th;   // angle from global y-axis about z-axis
    float velX; // velocity along global x-axis
    float velY; // velocity along global y-axis
    float w;    // angular velocity about z-axis
    float acc;  // acceleration along robot y-axis
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