/*

Filename: comms.hpp
Author: Jacob Boyer
Description: Header file for comms.cpp

*/

#ifndef COMMS_HPP
#define COMMS_HPP

#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_wifi.h"
#include "globals.hpp"

// Function to initialize the wifi utility
void initWifiSta();

// Function to handle mqtt events properly as they arise
void mqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);

// Function to start the mqtt connection
void mqttStart(const char* broker_ip);

// Function to publish the telemetry
void mqttPublishTelemetry(void* payload, size_t len);

// Function to initialize all comms utilities
void initComms();

#endif