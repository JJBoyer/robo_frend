/*

Project Name: Robo Frend
Description: An ESP32-based, dual-motor robot 
designed to accept a variety of attachments for 
use around an indoor space

File Name: main.cpp
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
#include "pwm_control.hpp"
#include "sensor_control.hpp"

using namespace std;

// ESP32 requires use of external C-based app_main loop
extern "C" void app_main() {

    // Initialize peripherals
    initSensors();
    initPWM();

    // Start task threads
    initTasks();

}