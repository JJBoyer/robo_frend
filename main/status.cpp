/*

File Name: status.cpp
Author: Jacob Boyer
Description: Defines getStatus(), a customizable
debugging function that writes data to the serial
monitor for troubleshooting and tuning

*/

#include "status.hpp"
#include "esp_log.h"

using namespace std;

/* getStatus:
  Collects data necessary to provide the desired
  information, then outputs the information to the
  serial monitor.
*/
void getStatus(){

    // Get information from other tasks

    // Initialize variables to store info
    int duty = 512;
    int freq = 500;
    double distance = 1.0;  // cm

    // Retrieve info from global pointers
    if(xSemaphoreTake(dutyMutex, pdMS_TO_TICKS(10))){
        duty = *pduty;
        xSemaphoreGive(dutyMutex);
    }

    if(xSemaphoreTake(freqMutex, pdMS_TO_TICKS(10))){
        freq = *pfreq;
        xSemaphoreGive(freqMutex);
    }

    if(xSemaphoreTake(distMutex, pdMS_TO_TICKS(10))){
        distance = *pdist;
        xSemaphoreGive(distMutex);
    }

    // Print Status to Serial Monitor

    // Clear monitor and set cursor to top-left
    printf("\033[2J\033[H");

    printf("--Sensor Input--\n");
    printf("Potentiometer Mode: Brightness\n");
    printf("Ultrasonic Mode: Frequency\n");
    printf("Distance: %lf cm\n\n", distance);

    printf("--LED Output--\n");
    printf("Frequency: %d\n", freq);
    printf("Duty Cycle: %d\n\n", duty);

}