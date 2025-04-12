/*



*/

#include "comms.hpp"
#include "esp_log.h"

using namespace std;

esp_mqtt_client_handle_t client = NULL;

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

void mqttStart(const char* broker_ip){
    
    char uri[64];
    snprintf(uri, sizeof(uri), "mqtt://%s", broker_ip);

    esp_mqtt_client_config_t mqtt_conf{};
    mqtt_conf.broker.address.uri = uri;

    client = esp_mqtt_client_init(&mqtt_conf);
    esp_mqtt_client_register_event(client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID), mqttEventHandler, NULL);
    esp_mqtt_client_start(client);
}

void mqttPublishTelemetry(void* payload, size_t len){
    if(client){
        esp_mqtt_client_publish(client, "robot/telemetry", (const char*)payload, len, 1, 0);
    }
}

void initComms(){
    nvs_flash_init();
    initWifiSta();
    mqttStart(BROKER_IP);
}