/*

Project Name: Robo Frend
Description: An ESP32-based, dual-motor robot 
designed to accept a variety of attachments for 
use around an indoor space

Filename: main.cpp
Author: Jacob Boyer
Description: Main loop for Robo Frend

*/

/* Project #include Standard
  
  The project standard for
  include order is as follows:

  - C/C++ Standard Libraries
  - ESP-IDF Libraries
  - Project Header Files

*/

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "tasks.hpp"
#include "task_manager.hpp"
#include "motor_control.hpp"
#include "sensor_control.hpp"
#include "globals.hpp"

using namespace std;

// ESP32 requires use of external C-based app_main loop
extern "C" void app_main() {

    // Initialize global assets
    initGlobals();

    // Initialize peripherals
    initSensors();
    initMotors();

    // Start task threads
    initTasks();

    // Readout system status
    printTaskStatus();

}