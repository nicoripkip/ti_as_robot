#include "network.hpp"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <time.h>
#include "config.hpp"
#include "PubSubClient.h"
#include "buffers.hpp"
#include <esp_wifi.h>


// Static globals needed for this file
WiFiClient      wifi_client;
PubSubClient    mqtt_client;
bool            mqtt_connected;
bool            wifi_connected;


/**
 * @brief Callback function to capture all mqtt data from the server
 * 
 * @param topic
 * @param payload
 * @param length
 */
void mqtt_callback(const char* topic, byte* payload, unsigned int length)
{
    if (strcmp(topic, "/hmi2robot") == 0) {
        Serial.println("Message received from hmi!");
    } else if (strcmp(topic, "/hypervisor2robot") == 0) {
        Serial.println("Message received from hypervisor!");
    }
}


/**
 * @brief Function to detect the macaddress of the ESP32
 * 
 */
void read_mac_address(){
    uint8_t baseMac[6];
    
    // Get MAC address of the WiFi station interface
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    Serial.print("Station MAC: ");
    for (int i = 0; i < 5; i++) {
      Serial.printf("%02X:", baseMac[i]);
    }
    Serial.printf("%02X\n", baseMac[5]);
}


/**
 * @brief Function to initialize the wifi network
 * 
 */
void init_Wifi() 
{
    read_mac_address();
    wifi_connected = false;

    // Before the loop can start, it is required that a stable network connection is needed!
    // WiFi.setMinSecurity(WIFI_AUTH_WEP); 
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Serial.print("Wifi is trying to connect to the network: ");

    // Try to connect to the network
    uint8_t counter = 0;
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        counter++;
        delay(500);

        if (counter >= 100) return;
    }

    // Serial.println("");
    // Serial.println("Wifi connected to network!");

    wifi_connected = true;

    // Turn energy down of the wifi
    // esp_wifi_set_max_tx_power(20);

    // Sync time with the time server
    // Serial.println("Trying to sync time!");
    configTime(0, (3600 * 2), "ntppool1.time.nl", "ntppool2.time.nl");
}


/**
 * @brief Function to initialize the mqtt server before it can be used
 * 
 */
void init_mqtt()
{
    mqtt_connected = false;

    // Set basic settings of mqtt
    mqtt_client.setClient(wifi_client);
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt_client.setCallback(mqtt_callback);

    // Serial.println("Try to connect to mqtt server!");
    if (!MQTT_CLIENT_ANONYMOUS) mqtt_connected = mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
    else mqtt_connected = mqtt_client.connect(MQTT_CLIENT_ID);

    // if (mqtt_connected) Serial.println("Succesfully connected to mqtt server");
    // else Serial.println("Failed to connect to mqtt server!");

    // Subscribe to the rx channels
    mqtt_client.subscribe("/ti/as/hmi2robot");
    mqtt_client.subscribe("/ti/as/hypervisor2robot");

    // Setup MQTT buffers
    mqtt_data_queue = xQueueCreate(10, sizeof(char) * 256);
    logger_queue = xQueueCreate(100, sizeof(char) * 256);
}


/**
 * @brief Function to connect to the internet and process any network activity
 * 
 * @param param
 */
void network_task(void *param)
{ 
    init_Wifi();
    if (wifi_connected) init_mqtt();

    while (true) {
        if (wifi_connected && mqtt_connected) {
            char message[256];
            memset(message, 0, 256);

            // Process of sending data 
            if (uxQueueMessagesWaiting(mqtt_data_queue) > 0) {
                xQueueReceive(mqtt_data_queue, &message, 50);

                mqtt_client.publish("/robot", message);
            }

            // Make sure client is always looped
            mqtt_client.loop();
        }
    }
}


