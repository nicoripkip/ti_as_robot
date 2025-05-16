#include "network.hpp"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <time.h>
#include "config.hpp"
#include "PubSubClient.h"


// Static globals needed for this file
WiFiClient      wifi_client;
PubSubClient    mqtt_client;
bool            mqtt_connected;


/**
 * @brief Callback function to capture all mqtt data from the server
 * 
 * @param topic
 * @param payload
 * @param length
 */
void mqtt_callback(const char* topic, byte* payload, unsigned int length)
{

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
    // Before the loop can start, it is required that a stable network connection is needed!
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Sync time with the time server
    Serial.println("Trying to sync time!");
    configTime(0, (3600 * 2), "ntppool1.time.nl", "ntppool2.time.nl");

    Serial.print("Wifi is trying to connect to the network: ");

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    Serial.println("");
    Serial.println("Wifi connected to network!");
}


/**
 * @brief Function to initialize the mqtt server before it can be used
 * 
 */
void init_mqtt()
{
    mqtt_connected = false;

    mqtt_client.setClient(wifi_client);
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt_client.setCallback(mqtt_callback);

    Serial.println("Try to connect to mqtt server!");
    if (!MQTT_CLIENT_ANONYMOUS) mqtt_connected = mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
    else mqtt_connected = mqtt_client.connect(MQTT_CLIENT_ID);

    if (mqtt_connected) Serial.println("Succesfully connected to mqtt server");
    else Serial.println("Failed to connect to mqtt server!");
}


/**
 * @brief Function to connect to the internet and process any network activity
 * 
 * @param param
 */
void network_task(void *param)
{ 
    init_Wifi();
    init_mqtt();

    while (true) {
        if (mqtt_connected) {

        }
    }
}


