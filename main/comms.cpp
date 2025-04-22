/*

Filename: comms.cpp
Author: Jacob Boyer
Description: Contains functions for the initialization
and use of Wi-Fi and MQTT connections for transmitting
telemetry to the base station.

*/

#include "comms.hpp"
#include "esp_log.h"

using namespace std;

esp_mqtt_client_handle_t client = NULL;

/* initWifiSta:
    Defines wifi config and connects to the desired wifi network.
*/
void initWifiSta(){
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t conf = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&conf);

    wifi_config_t wifi_conf{};
    strcpy((char*)wifi_conf.sta.ssid, "Jacob's iPhone");
    strcpy((char*)wifi_conf.sta.password, "CaboVerde");

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_conf);
    esp_wifi_start();
    esp_wifi_connect();
}

/* mqttEventHandler:
    Receives event ids from the mqtt protocol and
    prints status updates.
*/
void mqttEventHandler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("MQTT connected\n");
            break;
        case MQTT_EVENT_DISCONNECTED:
            printf("MQTT disconnected\n");
            break;
        default:
            break;
    }
}

/* mqttStart:
    Defines the mqtt config and starts the client
    to connect with the broker.
*/
void mqttStart(const char* broker_ip){
    
    char uri[64];
    snprintf(uri, sizeof(uri), "mqtt://%s", broker_ip);

    esp_mqtt_client_config_t mqtt_conf{};
    mqtt_conf.broker.address.uri = uri;

    client = esp_mqtt_client_init(&mqtt_conf);
    esp_mqtt_client_register_event(client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID), mqttEventHandler, NULL);
    esp_mqtt_client_start(client);
}

/* mqttPublishTelemetry:
    Takes in a payload packet and casts it to a c-string
    for transmission to the "robot/telemetry" topic.
*/
void mqttPublishTelemetry(void* payload, size_t len){
    if(client){
        esp_mqtt_client_publish(client, "robot/telemetry", (const char*)payload, len, 1, 0);
    }
}

/* initComms:
    Runs all initialization commands necessary for
    establishing Wi-Fi and MQTT connections
*/
void initComms(){
    nvs_flash_init();
    initWifiSta();
    mqttStart("192.168.50.117");
}