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

/* blinkLedTask:
  Simple task designed to run the blink()
  function, which receives a frequency and
  a duty cycle from the setFreq() and setBright()
  functions and drives the LED at those values

  CPU Core: 1
  Task Frequency: 10Hz
*/
void blinkLedTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // Sets task frequency to 10Hz

    while(true){
        blink();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 100ms has passed from task start
    }
}

/* setLEDTask:
  Simple task designed to run the setBright()
  function, which reads the ADC input from the
  potentiometer and sets the duty cycle of the
  PWM output based on the ADC reading

  CPU Core: 0
  Task Frequency: 10Hz
*/
void setLEDTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // Sets task frequency to 10Hz 

    while(true){
        setBright();
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
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Set task frequency to 20Hz

    while(true){
        setFreq();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 50ms has passed from task start
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
