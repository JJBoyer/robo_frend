#include "status.hpp"
#include "esp_log.h"

using namespace std;

void getStatus(){

    // Get information from other tasks

    // Initialize variables to store info
    int duty = 512;
    int freq = 500;
    double distance = 1.0;  // cm

    // Retrieve info from global pointers
    if(xSemaphoreTake(dutyMutex, pdMS_TO_TICKS(10))){
        duty = *pduty;
        //printf("Duty updated\n");
        xSemaphoreGive(dutyMutex);
    }

    if(xSemaphoreTake(freqMutex, pdMS_TO_TICKS(10))){
        freq = *pfreq;
        //printf("Freq updated\n");
        xSemaphoreGive(freqMutex);
    }

    if(xSemaphoreTake(distMutex, pdMS_TO_TICKS(10))){
        distance = *pdist;
        //printf("Distance updated\n");
        xSemaphoreGive(distMutex);
    }

    // Print Status to Serial Monitor

    // Prep monitor for display
    printf("\033[2J\033[H");  // Clear monitor and set cursor to top-left

    // Readout Sensor Status
    printf("--Sensor Input--\n");
    printf("Potentiometer Mode: Brightness\n");
    printf("Ultrasonic Mode: Frequency\n");
    printf("Distance: %lf\n\n", distance);

    // Readout PWM Outputs
    printf("--LED Output--\n");
    printf("Frequency: %d\n", freq);
    printf("Duty Cycle: %d\n\n", duty);

}