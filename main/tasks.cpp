/*

Filename: tasks.cpp
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
  function, which sets the motors
  to turn forward at full speed as simple
  motor control. Will be updated to run
  motors at specific speeds and directions
  once PD control is implemented.

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
  Simple task designed to run the getDistance()
  function, which takes the distance measured
  by the ultrasonic sensor and sets the blink
  frequency based on the distance.

  CPU Core: 0
  Task Frequency: 10Hz
*/
void ultrasonicTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Set task frequency to 10Hz

    while(true){
        getDistance();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 100ms has passed from task start
    }
}

/* encoderTask:
    Simple task to run the measureVelocity function, which
    measures the motor speed from the encoders on either wheel.

    CPU Core: 0
    Task Frequency: 10Hz
*/
void encoderTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Set task frquency to 10Hz

    while(true){
        measureVelocity();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait until 100ms has passed from task start
    }
}

/* estimateStateTask:
    Simple task designed to run the estimateState() function, 
    which reads accelerometer and gyro data from the MPU6050 
    and uses the data to estimate the state of the robot at
    fixed time steps.

    CPU Core: 0
    Task Frequency: 40Hz
*/
void estimateStateTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Set task frequency to 40Hz

    while(true){
        estimateState(mpu_sensor);
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until 25ms has passed from task start
    }
}

/* purePursuitControlTask:

*/
void purePursuitControlTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();  // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // Set task frequency to 40 Hz

    while(true){
        purePursuitControl();
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait until 25ms has passed from task start
    }
}

/* sendTelemetryTask:
  Simple task designed to run the sendTelemetry()
  function, which compiles system data and transmits
  it to the base station via MQTT protocol.

  CPU Core: 0
  Task Frequency: 0.2 Hz
*/
void sendTelemetryTask(void* pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();    // Initialize last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(5000); // Set task frequency to 0.2Hz

    while(true){
        sendTelemetry();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);   // Wait until 5s has passed from task start
    }
}
