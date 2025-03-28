/*

File Name: tasks.cpp
Author: Jacob Boyer
Description: Define tasks to facilitate
individual robot operations at definite
intervals. Each task should include one
function representing an operation and 
the timing interval at which it should
be run.

*/

#include "tasks.hpp"
#include "esp_log.h"

using namespace std;

/* motorTask:
  Simple task designed to run the forward()
  function, which receives sets the motors
  to turn forward at full speed as simple
  motor control

  CPU Core: 1
  Task Frequency: 10Hz
*/
void motorTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // Sets task frequency to 10Hz

    while(true){
        forward();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 100ms has passed from task start
    }
}

/* ultrasonicTask:
  Simple task designed to run the setFreq()
  function, which takes the distance measured
  by the ultrasonic sensor and sets the blink
  frequency based on the distance.

  CPU Core: 0
  Task Frequency: 20Hz
*/
void ultrasonicTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Set task frequency to 10Hz

    while(true){
        getDistance();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 50ms has passed from task start
    }
}

/* estimateStateTask:
    Defines a task that reads data from the MPU6050 and
    uses the data to estimate the state of the robot at
    fixed time steps.
*/
void estimateStateTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Set task frequency to 40Hz

    while(true){
        estimateState(mpu_sensor);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 25ms has passed from task start
    }
}

/* getStatusTask:
  Simple task designed to run the getStatus()
  function, which writes any data desired for
  tuning or debugging to the Serial Monitor

  CPU Core: 0
  Task Frequency: 0.2 Hz
*/
void getStatusTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Offset task to avoid interference with readPot
    const TickType_t xFrequency = pdMS_TO_TICKS(5000);  // Set task frequency to 0.2Hz

    while(true){
        // Display remaining words in the stack (1 word = 1 int = 4 bytes)
        getStatus();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // run task at ~0.2Hz
    }
}
