/*

Filename: telemetry.cpp
Author: Jacob Boyer
Description: Defines sendTelemetry(), which compiles
a telemetry struct and transmits it to the base station

*/

#include "telemetry.hpp"
#include "esp_log.h"
#include "mutex_guard.hpp"

using namespace std;

/* getStatus:
  Collects data necessary to provide the desired
  feedback, then outputs the information to the
  serial monitor.
*/
void sendTelemetry(){

    // Get feedback from other tasks

    // Initialize variables to store info
    struct {
        motorOut_t duty;
        state_t state;
    } telemetry;

    // Retrieve info from global pointers
    {
        MutexGuard lock(dutyMutex);
        telemetry.duty = *pduty;
    }
    
    {
        MutexGuard lock(stateMutex);
        telemetry.state = *pstate;
    }

    mqttPublishTelemetry(&telemetry, sizeof(telemetry));

}