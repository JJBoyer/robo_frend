/*

File Name: sensor_control.cpp
Author: Jacob Boyer
Description: Defines a function to initialize the
sensors for robot control, then defines functions
necessary for measuring obstacle distances and IMU
data for position extrapolation.

*/

#include "sensor_control.hpp"
#include "esp_log.h"
#include "mutex_guard.hpp"

using namespace std;

// Initialize volatile variables for use in ISR
volatile int64_t start_time = 0;
volatile int64_t stop_time = 0;
volatile bool measurement_ready = false;

// Create an ADC handle
adc_oneshot_unit_handle_t adc1_handle;

/* echo_isr_handler:
  On a change interrupt (rising or falling edge) from the
  echo pin, this handler collects a system time value. On 
  a rising edge, the time is stored as the start of the signal,
  while on a falling edge the time is stored as the end of
  the signal. These times are used to calculate distance in
  getDistance() one the measurement_ready flag is raised.
*/
void IRAM_ATTR echo_isr_handler(void* arg){
    if (gpio_get_level(ECHO) == 1) {
        start_time = esp_timer_get_time(); // Get timestamp (in microseconds)
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

    initADC();
    initSR04();
    //initI2C(); // to be added

}

/* initADC:
  Configures and initializes the ADC and
  any desired ADC GPIO pins.
*/
void initADC(){

    // ADC Config
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,    // RTC clock source (0 uses default)
        .ulp_mode = ADC_ULP_MODE_DISABLE        // Disable ULP mode
    };
    // Implement ADC config
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if(ret != ESP_OK){
        printf("ERROR: Failed to initialize ADC1! Error Code: %d\n", ret);
        return;
    }

    // ADC channel config
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    // Implement ADC channel config on READ_PIN
    esp_err_t chan_ret = adc_oneshot_config_channel(adc1_handle, READ_PIN, &config);
    if(chan_ret != ESP_OK){
        printf("ERROR: Failed to configure ADC channel! Error Code: %d\n", chan_ret);
        return;
    }

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

    // Configure HC-SR04 test pin
    gpio_config_t check_conf = {
        .pin_bit_mask = (1ULL << CHECK), // Set pin assignment
        .mode = GPIO_MODE_INPUT        // Set to output
    };
    // Implement trigger config
    gpio_config(&check_conf);

    // Install ISR service and attach interrupt
    gpio_install_isr_service(0);
    esp_err_t isr_error = gpio_isr_handler_add(ECHO, echo_isr_handler, NULL);
    if(isr_error != ESP_OK){
        printf("Error: ISR Not Attached. Code: %d\n", isr_error);
    }

}
/* (WIP) initI2C:
  Initializes the I2C bus using user-defined GPIO pins.
  NOTE: CURRENTLY EMPTY
*/
void initI2C(){}

/* setBright:
  Reads from the potentiometer and writes to
  the global pointer for duty cycle, setting
  the brightness of the LED, for now.
*/
void setBright(){

    // Collect pot data
    int data = readPot();

    // Write new duty cycle to duty pointer
    {
        MutexGuard lock(dutyMutex);
        pduty.reset(new int(1100 - data));
    }

}

/* setFreq:
  Reads from the global pointer for distance, then
  writes to the global pointer for frequency,
  setting the blink frequency of the LED
*/
void setFreq(){

    // Initialize utility variables
    static double distance = 1.0;
    static int frequency = 100;

    // Ping ultrasonic sensor
    getDistance();

    // Get ultrasonic distance to object
    {
        MutexGuard lock(distMutex);
        distance = *pdist;
    }

    // Calculate frequency input
    frequency = 100 * distance;

    // Saturate frequency
    if(frequency > 1500){
        frequency = 1500;
    }

    // Set new LED frequency
    {
        MutexGuard lock(freqMutex);
        pfreq.reset(new int(frequency));
    }

}

/* getDistance:
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
    if(time == 1458){
        printf("Distance is less than %lf cm.\n", distance);
    } else {
        printf("Distance: %lf cm\n", *pdist);
    }

    // Set distance pointer to current distance
    {
        MutexGuard lock(distMutex);
        pdist.reset(new double(distance));
    }

}

/* readPot:
  Reads the voltage from the potentiometer through
  the ADC, then converts the ADC output into a value
  between 100 and 1000.
*/
int readPot(){

    // Declare variable to store the raw ADC output
    int raw;

    // Convert the data into a value between 100 and 1000
    adc_oneshot_read(adc1_handle, READ_PIN, &raw);
    double vin = raw * 3.3 / 4095;
    int data = 1000 / (vin * 10 / 3.3);

    // Saturate data at 1000 if it would otherwise be greater
    if(data > 1000){
        data = 1000;
    }

    return data;

}
