/*

Filename: sensor_control.cpp
Author: Jacob Boyer
Description: Defines a function to initialize the
sensors for robot control, then defines functions
necessary for measuring obstacle distances and IMU
data for position extrapolation.

*/

#include "driver/i2c.h"
#include "esp_log.h"
#include "sensor_control.hpp"
#include "encoder.hpp"
#include "imu.hpp"
#include "mutex_guard.hpp"

using namespace std;

// Initialize volatile variables for use in ISR
volatile int64_t start_time = 0;
volatile int64_t stop_time = 0;
volatile bool measurement_ready = false;

/* echo_isr_handler:
  On a change interrupt (rising or falling edge) from the
  echo pin, this handler collects a system time value. On 
  a rising edge, the time is stored as the start of the signal,
  while on a falling edge the time is stored as the end of
  the signal. These times are used to calculate distance in
  getDistance() one the measurement_ready flag is raised.
*/
void IRAM_ATTR echo_isr_handler(void* arg){
    // Get timestamp (in microseconds)
    if (gpio_get_level(ECHO) == 1) {
        start_time = esp_timer_get_time();
    } else {
        stop_time = esp_timer_get_time();
        measurement_ready = true;
    }
}

/* initSensors:
  Bundles all sensor initialization functions
  into one function to be called in main.cpp
*/
void initSensors(){

    initI2C();
    initSR04();
    init6050();
    initEncoders();

}

/* initSR04:
  Configures the HC-SR04 trigger and echo pins.
*/
void initSR04(){

    // Configure HC-SR04 trigger pin
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG), // Set pin assignment
        .mode = GPIO_MODE_OUTPUT        // Set to output
    };
    // Implement trigger config
    gpio_config(&trig_conf);

    // Configure HC-SR04 echo pin
    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO), // Set pin assignment
        .mode = GPIO_MODE_INPUT,        // Set to input
        .intr_type = GPIO_INTR_ANYEDGE  // Interrupt on both rising and falling edges
    };
    // Implement echo config
    gpio_config(&echo_conf);

    // Install ISR service and attach interrupt
    gpio_install_isr_service(0);
    esp_err_t isr_error = gpio_isr_handler_add(ECHO, echo_isr_handler, NULL);
    if(isr_error != ESP_OK){
        printf("Error: ISR Not Attached. Code: %d\n", isr_error);
        return;
    }

    status.set(ULTRASONIC);
}

/* initI2C:
  Initializes the I2C bus using user-defined GPIO pins.
*/
void initI2C(){

    // Configure I2C with custom pins
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000
        }
    };

    // Apply config and install driver
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

}

/* getDistance: NOTE - adapt to obstacle avoidance
  Activates the HC-SR04 ultrasonic sensor, then
  measures the time taken for the signal to reflect
  from the nearest object. Then, the distance to the
  object is calculated by multiplying the travel time
  by the speed of sound (343m/s) and dividing by 2 to
  account for the sound travelling both to and from
  the object. The distance is then written to the global
  distance pointer.
*/
void getDistance(){

    double distance;

    // Trigger Sensor - 10us HIGH signal to TRIG pin
    gpio_set_level(TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG, 0);

    // Wait for measurement to be ready
    int cycles = 0;
    while(!measurement_ready){
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to free CPU while waiting
        if(++cycles >= 50){
            printf("Warning: Echo Timeout\n");
            return;
        }
    }
    measurement_ready = false;

    // Calculate total travel time
    int64_t time = stop_time - start_time;  // Gives travel time in microseconds

    // Saturate the minimum value of travel time to block out short range errors
    if(time < 1458){
        time = 1458;  // microseconds
    }

    // Calculate distance
    distance = time * 0.0343 / 2;  // time * [speed of sound] / 2

    // Set distance pointer to current distance
    {
        MutexGuard lock(distMutex);
        pdist.reset(new double(distance));
    }

}

