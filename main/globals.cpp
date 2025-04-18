/*

Filename: globals.cpp
Author: Jacob Boyer
Description: A file designed to improve the scalability of the inter-task
communication, allowing each pointer and corresponding mutex to be defined
once in this file and included in all files that use it.

*/

#include "globals.hpp"
#include <stdlib.h>
#include "esp_log.h"
#include "mutex_guard.hpp"

using namespace std;

// Allocate status bitset memory
bitset<FLAG_COUNT> status;

// Allocate global pointer memory
unique_ptr<motorOut_t> pduty = nullptr;
unique_ptr<double> pdist = nullptr;
unique_ptr<odometry_t> pwheel = nullptr;
unique_ptr<state_t> pstate = nullptr;
unique_ptr<queue<Pose2D_t>> pway = nullptr;
unique_ptr<vector<pathPoint_t>> ppath = nullptr;

// Allocate memory for mutexes
SemaphoreHandle_t dutyMutex  = nullptr;
SemaphoreHandle_t distMutex  = nullptr;
SemaphoreHandle_t wheelMutex = nullptr;
SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t wayMutex   = nullptr;
SemaphoreHandle_t pathMutex  = nullptr;

// Define constant system parameters
const float velocityTarget = 0.2f;
const float wheelBaseWidth = 0.15f;

void initGlobals() {

    // Initialize mutexes for pointer locking
    dutyMutex  = xSemaphoreCreateMutex();
    distMutex  = xSemaphoreCreateMutex();
    wheelMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    wayMutex   = xSemaphoreCreateMutex();
    pathMutex  = xSemaphoreCreateMutex();
    

    // Initialize global pointers
    { // dutyMutex
        MutexGuard lock(dutyMutex);
        pduty.reset(new motorOut_t{0, 0});
        if(pduty == nullptr){
            return;
        }
    }

    { // stateMutex
        MutexGuard lock(stateMutex);
        pstate.reset(new state_t({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}));
        if(pstate == nullptr){
            return;
        }
    }

    { // wheelMutex
        MutexGuard lock(wheelMutex);
        pwheel.reset(new odometry_t({0, 0.0f, 0.0f, 0.0f, 0.0f}));
        if(pwheel == nullptr){
            return;
        }
    }

    { // distMutex
        MutexGuard lock(distMutex);
        pdist.reset(new double(0.0));
        if(pdist == nullptr){
            return;
        }
    }

    { // wayMutex
        MutexGuard lock(wayMutex);
        pway.reset(new queue<Pose2D_t>());
        if(pway == nullptr){
            return;
        }
    }

    { // pathMutex
        MutexGuard lock(pathMutex);
        ppath.reset(new vector<pathPoint_t>());
        if(ppath == nullptr){
            return;
        }
    }

    status.set(GLOBALS);
}