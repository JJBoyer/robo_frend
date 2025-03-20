// Include libraries as needed
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Include own header files
#include "tasks.hpp"
#include "pwm_control.hpp"
#include "sensor_control.hpp"

using namespace std;  // Use the std namespace

// use external C-based app_main function as main loop for ESP32
extern "C" void app_main() {

    // Initialize peripherals
    initPWM();    // See pwm_control.cpp for more details
    initSensors();    // See sensor_contril.cpp for more details
    //initI2C();  // To be included in future update

    // Start task threads
    initTasks();

    // ADC task runs on Core 0 at 10Hz
    // Status task runs on Core 0 at 0.2Hz
    // PWM task runs on Core 1 at 10Hz

}