#include "tasks.hpp"
#include "esp_log.h"

using namespace std;

// Blink task definition
void blinkLedTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // Sets task frequency to 10Hz

    while(true){
        blink();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 100ms has passed from task start
    }
}

// Read pot task definition
void setLEDTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // Sets task frequency to 10Hz 

    while(true){
        setBright();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 100ms has passed from task start
    }
}

// Poll ultrasonic sensor for regular distance data
void ultrasonicTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Set task frequency to 100Hz

    while(true){
        setFreq();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 10ms has passed from task start
    }
}

// Get status task definition
void getStatusTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Offset task to avoid interference with readPot
    const TickType_t xFrequency = pdMS_TO_TICKS(5000);  // Set task frequency to 0.2Hz

    while(true){
        // Display remaining words in the stack (1 word = 1 int = 4 bytes)
        getStatus();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // run task at ~0.2Hz
    }
}
