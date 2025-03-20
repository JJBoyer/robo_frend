#include "sensor_control.hpp"
#include "esp_log.h"

using namespace std;

// Initialize volatile variables for use in ISR
volatile int64_t start_time = 0;
volatile int64_t stop_time = 0;
volatile bool measurement_ready = false;

// Create an ADC handle
adc_oneshot_unit_handle_t adc1_handle;

// Setup interrupt handler for ultrasonic sensor
void IRAM_ATTR echo_isr_handler(void* arg){
    if (gpio_get_level(ECHO) == 1) {
        start_time = esp_timer_get_time(); // Get timestamp (in microseconds)
    } else {
        stop_time = esp_timer_get_time();
        measurement_ready = true;
    }
}

// Initialize ADC channel
void initSensors(){

    // Initialize HC-SR04 pins
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG), // Set pin assignment
        .mode = GPIO_MODE_OUTPUT        // Set to output
    };
    gpio_config(&trig_conf);  // Implement config

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO), // Set pin assignment
        .mode = GPIO_MODE_INPUT,        // Set to input
        .intr_type = GPIO_INTR_ANYEDGE  // Interrupt on both rising and falling edges
    };
    gpio_config(&echo_conf);  // Implement config

    // Install ISR service and attach interrupt
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ECHO, echo_isr_handler, NULL);

    // ADC Config
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,    // RTC clock source (0 uses default)
        .ulp_mode = ADC_ULP_MODE_DISABLE        // Disable ULP mode
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if(ret != ESP_OK){
        printf("ERROR: Failed to initialize ADC1! Error Code: %d\n", ret);
        return;
    }

    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    esp_err_t chan_ret = adc_oneshot_config_channel(adc1_handle, READ_PIN, &config);
    if(chan_ret != ESP_OK){
        printf("ERROR: Failed to configure ADC channel! Error Code: %d\n", chan_ret);
        return;
    }

}

void setBright(){

    // Collect pot data
    int data = readPot();

    if(xSemaphoreTake(dutyMutex, portMAX_DELAY)){
        pduty.reset(new int(1100 - data));
        xSemaphoreGive(dutyMutex);
    }

}

void setFreq(){

    // Initialize utility variables
    static double distance = 1.0;
    static double backup = 1.0;
    static int frequency = 100;

    // Get ultrasonic distance to object
    if(xSemaphoreTake(distMutex, pdMS_TO_TICKS(portMAX_DELAY))){
        distance = *pdist;
        backup = distance;
        xSemaphoreGive(distMutex);
    } else {
        distance = backup;
    }

    // Calculate frequency input
    frequency = 100 * distance;

    // Saturate frequency
    if(frequency > 1500){
        frequency = 1500;
    }

    // Set new LED frequency
    if(xSemaphoreTake(freqMutex, portMAX_DELAY)){
        pfreq.reset(new int(frequency));
        xSemaphoreGive(freqMutex);
    }

}

// Get distance from SR04 Ultrasonic Sensor
void getDistance(){

    // Trigger Sensor
    // 10us HIGH signal to TRIG pin
    gpio_set_level(TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG, 0);

    // Wait for measurement to be ready
    while(!measurement_ready){
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to free CPU while waiting
    }

    // Calculate total travel time
    int64_t time = stop_time - start_time;  // Gives travel time in microseconds

    // Set distance pointer to current distance
    if(xSemaphoreTake(distMutex, portMAX_DELAY)){
        pdist.reset(new double(time * 0.0343 / 2));  // Dist = time * VelSound / 2
        xSemaphoreGive(distMutex);
    }

}

// Read from potentiometer input
int readPot(){

    int raw;

    adc_oneshot_read(adc1_handle, READ_PIN, &raw);  // read analog input from READ_PIN
    double vin = raw * 3.3 / 4095;  // convert the raw ADC output to an input voltage
    int data = 1000 / (vin * 10 / 3.3);  // map the input voltage to a set of periods

    if(data > 1000){
        data = 1000;
    }

    return data;

}
