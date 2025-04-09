/*

Filename: status.cpp
Author: Jacob Boyer
Description: Defines getStatus(), a customizable
debugging function that writes data to the serial
monitor for troubleshooting and tuning

*/

#include "status.hpp"
#include "esp_log.h"
#include "mutex_guard.hpp"

using namespace std;

/* getStatus:
  Collects data necessary to provide the desired
  feedback, then outputs the information to the
  serial monitor.
*/
void getStatus(){

    // Get feedback from other tasks

    // Initialize variables to store info
    motorOut_t duty = {
        .left  = 512,
        .right = 512
    };
    double accel = 1.0;
    double distance = 1.0;  // cm

    // Retrieve info from global pointers
    {
        MutexGuard lock(dutyMutex);
        duty = *pduty;
    }
    
    {
        MutexGuard lock(stateMutex);
        accel = 9.81 * (*pstate).acc;
    }

    {
        MutexGuard lock(distMutex);
        distance = *pdist;
    }

    // Print Status to Serial Monitor

    // Clear monitor and set cursor to top-left
    /*printf("\033[2J\033[H");

    printf("--Sensor Input--\n");
    printf("Potentiometer Mode: Brightness\n");
    printf("Ultrasonic Mode: Frequency\n");
    printf("Distance: %lf cm\n\n", distance);

    printf("--Motor Output--\n");
    printf("Acceleration: %lf m/s^2\n", accel);
    printf("Left Duty Cycle: %d\n", duty.left);
    printf("Right Duty Cycle: %d\n", duty.right);
    */
}