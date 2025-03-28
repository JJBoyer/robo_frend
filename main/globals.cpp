#include "globals.hpp"
#include "esp_log.h"
#include <stdlib.h>
#include "mutex_guard.hpp"

using namespace std;

// Allocate global pointer memory
unique_ptr<int> pduty = nullptr;
unique_ptr<state_t> pstate = nullptr;
unique_ptr<double> pdist = nullptr;

// Allocate memory for mutexes
SemaphoreHandle_t dutyMutex = nullptr;
SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t distMutex = nullptr;

void initGlobals() {

    // Initialize mutexes for pointer locking
    dutyMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    distMutex = xSemaphoreCreateMutex();

    // Initialize global pointers
    {
        MutexGuard lock(dutyMutex);
        pduty.reset(new int(0));
    }

    {
        MutexGuard lock(stateMutex);
        pstate.reset(new state_t({0, 0, 0}));
    }

    {
        MutexGuard lock(distMutex);
        pdist.reset(new double(0.0));
    }

}