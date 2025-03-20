#include "task_manager.hpp"
#include "esp_log.h"

using namespace std;

// Initialize global pointers for access from all threads
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

// Initialize task threads
void initTasks(){

    // Initialize mutexes
    freqMutex = xSemaphoreCreateMutex();
    dutyMutex = xSemaphoreCreateMutex();
    distMutex = xSemaphoreCreateMutex();

    // Create FreeRTOS task for LED brightness and check
    BaseType_t result0 = xTaskCreatePinnedToCore(setLEDTask, "SetLED", 2048, NULL, 2, &setHandle, 0);
    if(result0 != pdPASS){
        printf("\nTask creation failed!\nTask: SetLED\nCPU: 0\n\n");
    }

    // Create FreeRTOS task for LED and check
    BaseType_t result1 = xTaskCreatePinnedToCore(blinkLedTask, "BlinkLED", 2048, NULL, 1, &blinkHandle, 1);
    if(result1 != pdPASS){
        printf("\nTask creation failed!\nTask: BlinkLED\nCPU: 1\n\n");
    }

    // Create FreeRTOS task for Ultrasonic Sensor reading
    BaseType_t result2 = xTaskCreatePinnedToCore(ultrasonicTask, "Ultrasonic", 2048, NULL, 3, &sonicHandle, 0);
    if(result2 != pdPASS){
        printf("\nTask creation failed!\nTask: Ultrasonic\nCPU: 0\n\n");
    }

    // Create FreeRTOS task for Status Updates and check (Create this last)
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