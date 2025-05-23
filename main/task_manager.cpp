/*

Filename: task_manager.cpp
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
TaskHandle_t encoderHandle = NULL;
TaskHandle_t estimateHandle = NULL;
TaskHandle_t pursuitHandle = NULL;
TaskHandle_t teleHandle = NULL;

/* Task CPU Core Assignments:
    Motor runs on Core 1 at 10Hz
    Ultrasonic runs on Core 0 at 10Hz
    StateEstimator runs on Core 0 at 40Hz
    GetStatus runs on Core 0 at 0.2Hz
*/
void initTasks(){

    // Initialize FreeRTOS task for controlling motor output
    BaseType_t result0 = xTaskCreatePinnedToCore(motorTask, "Motor", 2048, NULL, 1, &motorHandle, 1);
    if(result0 != pdPASS){
        printf("\nTask creation failed!\nTask: Motor\nCPU: 1\n\n");
        return;
    }

    // Temporarily deactivated
    // Initialize FreeRTOS task for measuring distance with the HC-SR04 ultrasonic sensor
    /*BaseType_t result1 = xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic", 4096, NULL, 3, &sonicHandle, 0);
    if(result1 != pdPASS){
        printf("\nTask creation failed!\nTask: Ultrasonic\nCPU: 0\n\n");
        return;
    }*/

    // Initialize FreeRTOS task for reading encoders to estimate robot state
    BaseType_t result2 = xTaskCreatePinnedToCore(encoderTask, "Encoders", 4096, NULL, 3, &encoderHandle, 0);
    if(result2 != pdPASS){
        printf("\nTask creation failed!\nTask: Encoders\nCPU: 0\n\n");
        return;
    }

    // Initialize FreeRTOS task for using MPU6050 to estimate robot state
    BaseType_t result3 = xTaskCreatePinnedToCore(estimateStateTask, "StateEstimator", 8192, mpu_sensor, 4, &estimateHandle, 0);
    if(result3 != pdPASS){
        printf("\nTask creation failed!\nTask: StateEstimator\nCPU: 0\n\n");
        return;
    }

    // Initialize FreeRTOS task for pure pursuit motor control
    BaseType_t result4 = xTaskCreatePinnedToCore(purePursuitControlTask, "PurePursuit", 8192, NULL, 1, &pursuitHandle, 0);
    if(result4 != pdPASS){
        printf("\nTask creation failed!\nTask: PurePursuit\nCPU: 0\n\n");
        return;
    }

    // Temporarily deactivated
    // Initialize FreeRTOS task for printing to the Serial Monitor
    /*BaseType_t resultn = xTaskCreatePinnedToCore(sendTelemetryTask, "SendTelemetry", 4096, NULL, 4, &teleHandle, 0);
    if(resultn != pdPASS){
        printf("\nTask creation failed!\nTask: GetStatus\nCPU: 0");
        return;
    }*/

    // Report successful initialization
    status.set(TASKS);
}

/* printTaskStatus:
    Measure the remaining stack space for each task.
    For use in optimizing memory allocation and identifying
    stack overflows.
*/
void printTaskStatus(){

    // Confirm successful initialization
    if(status.all()){
        printf("\nAll systems initialized successfully.\n\n");
    } else {
        printf("\nSystems failed to initialize:\n");
        if(!status.test(GLOBALS)){
            printf(" - Globals\n");
        }
        if(!status.test(ULTRASONIC)){
            printf(" - Ultrasonic\n");
        }
        if(!status.test(MPU6050)){
            printf(" - MPU6050\n");
        }
        if(!status.test(ENCODERS)){
            printf(" - Encoders\n");
        }
        if(!status.test(MOTORS)){
            printf(" - Motors\n");
        }
        if(!status.test(TASKS)){
            printf(" - Tasks\n");
        }
        printf("\n");
    }

    // Readout remaining space in each task
    printf("--Task Memory Use--\n");       
    if(motorHandle != NULL){
        printf("Motor remaining stack: %d words\n", uxTaskGetStackHighWaterMark(motorHandle));
    }
    if(sonicHandle != NULL){
        printf("Ultrasonic remaining stack: %d words\n", uxTaskGetStackHighWaterMark(sonicHandle));
    }
    if(encoderHandle != NULL){
        printf("Encoders remaining stack: %d words\n", uxTaskGetStackHighWaterMark(encoderHandle));
    }
    if(estimateHandle != NULL){
        printf("StateEstimator remaining stack: %d words\n", uxTaskGetStackHighWaterMark(estimateHandle));
    }
    if(pursuitHandle != NULL){
        printf("PurePursuit remaining stack: %d words\n", uxTaskGetStackHighWaterMark(pursuitHandle));
    }
    if(teleHandle != NULL){
        printf("SendTelemetry remaining stack: %d words\n\n", uxTaskGetStackHighWaterMark(teleHandle));
    }
}