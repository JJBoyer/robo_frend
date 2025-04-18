/*

Project Name: Robo Frend
Description: An ESP32-based, dual-motor robot 
designed to autonomously navigate increasingly
complex environments.

Filename: main.cpp
Author: Jacob Boyer
Description: Main loop for Robo Frend

*/

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "tasks.hpp"
#include "task_manager.hpp"
#include "motor_control.hpp"
#include "sensor_control.hpp"
#include "globals.hpp"
#include "comms.hpp"

using namespace std;

// ESP32 requires use of external C-based app_main loop
extern "C" void app_main() {

    // Initialize global assets
    initGlobals();

    // Initialize communication utilities
    initComms();

    // Initialize peripherals
    initSensors();
    initMotors();

    // Start task threads
    initTasks();

    // Readout system status
    printTaskStatus();

}