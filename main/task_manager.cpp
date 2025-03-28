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

// Initialize task handles
TaskHandle_t motorHandle = NULL;
TaskHandle_t sonicHandle = NULL;
TaskHandle_t estimateHandle = NULL;
TaskHandle_t statHandle = NULL;

/* Task CPU Core Assignments:
    SetLED runs on Core 0 at 10Hz
    Ultrasonic runs on Core 0 at 20Hz
    GetStatus runs on Core 0 at 0.2Hz
    BlinkLED runs on Core 1 at 10Hz
*/
void initTasks(){

    // Initialize FreeRTOS task for setting LED frequency
    BaseType_t result0 = xTaskCreatePinnedToCore(motorTask, "Motor", 2048, NULL, 1, &motorHandle, 1);
    if(result0 != pdPASS){
        printf("\nTask creation failed!\nTask: Motor\nCPU: 1\n\n");
    }

    // Initialize FreeRTOS task for measuring distance with the HC-SR04 ultrasonic sensor
    BaseType_t result1 = xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic", 4096, NULL, 3, &sonicHandle, 0);
    if(result1 != pdPASS){
        printf("\nTask creation failed!\nTask: Ultrasonic\nCPU: 0\n\n");
    }

    // Initialize FreeRTOS task for using MPU6050 to estimate robot state
    BaseType_t result2 = xTaskCreatePinnedToCore(estimateStateTask, "StateEstimator", 4096, mpu_sensor, 4, &estimateHandle, 0);
    if(result2 != pdPASS){
        printf("\nTask creation failed!\nTask: StateEstimator\nCPU: 0\n\n");
    }

    // Initialize FreeRTOS task for printing to the Serial Monitor
    BaseType_t resultn = xTaskCreatePinnedToCore(getStatusTask, "GetStatus", 4096, NULL, 4, &statHandle, 0);
    if(resultn != pdPASS){
        printf("\nTask creation failed!\nTask: GetStatus\nCPU: 0");
    }

}

// Display the stack usage of the tasks
void printTaskStatus(){

    // Readout remaining space in each task
    printf("--Task Memory Use--\n");       
    if(motorHandle != NULL){
        printf("Motor remaining stack: %d words\n", uxTaskGetStackHighWaterMark(motorHandle));
    }
    if(sonicHandle != NULL){
        printf("Ultrasonic remaining stack: %d words\n", uxTaskGetStackHighWaterMark(sonicHandle));
    }
    if(estimateHandle != NULL){
        printf("StateEstimator remaining stack: %d words\n", uxTaskGetStackHighWaterMark(estimateHandle));
    } 
    if(statHandle != NULL){
        printf("GetStatus remaining stack: %d words\n\n", uxTaskGetStackHighWaterMark(statHandle));
    }
}