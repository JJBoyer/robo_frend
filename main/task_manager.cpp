/*

File Name: task_manager.cpp
Author: Jacob Boyer
Description: Handles task thread initialization,
including inter-thread communication pointers,
data-lock mutexes, and individual tasks

*/

#include "task_manager.hpp"
#include "esp_log.h"

using namespace std;

/* Information transmitted between threads:
    pfreq -> blink frequency (determined by obstacle distance)
    pduty -> blink brightness (determined by potentiometer input)
    pdist -> obstacle distance (measured by ultrasonic sensor)
*/
unique_ptr<int> pfreq = make_unique<int>(500);
unique_ptr<int> pduty = make_unique<int>(512);
unique_ptr<double> pdist = make_unique<double>(1.00);

// Define mutexes
SemaphoreHandle_t freqMutex;
SemaphoreHandle_t dutyMutex;
SemaphoreHandle_t distMutex;

// Initialize task handles
TaskHandle_t setHandle = NULL;
TaskHandle_t blinkHandle = NULL;
TaskHandle_t sonicHandle = NULL;
TaskHandle_t statHandle = NULL;

/* Task CPU Core Assignments:
    SetLED runs on Core 0 at 10Hz
    Ultrasonic runs on Core 0 at 20Hz
    GetStatus runs on Core 0 at 0.2Hz
    BlinkLED runs on Core 1 at 10Hz
*/
void initTasks(){

    // Initialize mutexes for pointer locking
    freqMutex = xSemaphoreCreateMutex();
    dutyMutex = xSemaphoreCreateMutex();
    distMutex = xSemaphoreCreateMutex();

    // Initialize FreeRTOS task for setting LED brightness
    BaseType_t result0 = xTaskCreatePinnedToCore(setLEDTask, "SetLED", 2048, NULL, 2, &setHandle, 0);
    if(result0 != pdPASS){
        printf("\nTask creation failed!\nTask: SetLED\nCPU: 0\n\n");
    }

    // Initialize FreeRTOS task for setting LED frequency
    BaseType_t result1 = xTaskCreatePinnedToCore(blinkLedTask, "BlinkLED", 2048, NULL, 1, &blinkHandle, 1);
    if(result1 != pdPASS){
        printf("\nTask creation failed!\nTask: BlinkLED\nCPU: 1\n\n");
    }

    // Initialize FreeRTOS task for measuring distance with the HC-SR04 ultrasonic sensor
    BaseType_t result2 = xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic", 4096, NULL, 3, &sonicHandle, 0);
    if(result2 != pdPASS){
        printf("\nTask creation failed!\nTask: Ultrasonic\nCPU: 0\n\n");
    }

    // Initialize FreeRTOS task for printing to the Serial Monitor
    BaseType_t result3 = xTaskCreatePinnedToCore(getStatusTask, "GetStatus", 4096, NULL, 4, &statHandle, 0);
    if(result3 != pdPASS){
        printf("\nTask creation failed!\nTask: GetStatus\nCPU: 0");
    }

}

// Display the stack usage of the tasks
void printTaskStatus(){

    // Readout remaining space in each task
    printf("--Task Memory Use--\n");
    if(setHandle != NULL){
        printf("SetLED remaining stack: %d words\n", uxTaskGetStackHighWaterMark(setHandle));
    }        
    if(blinkHandle != NULL){
        printf("BlinkLED remaining stack: %d words\n", uxTaskGetStackHighWaterMark(blinkHandle));
    }
    if(sonicHandle != NULL){
        printf("Ultrasonic remaining stack: %d words\n", uxTaskGetStackHighWaterMark(sonicHandle));
    }
    if(statHandle != NULL){
        printf("GetStatus remaining stack: %d words\n\n", uxTaskGetStackHighWaterMark(statHandle));
    }

}