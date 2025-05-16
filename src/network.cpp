#include "network.hpp"
#include <Arduino.h>
#include <WiFi.h>
#include "config.hpp"


/**
 * @brief Function to detect the macaddress of the ESP32
 * 
 */
void readMacAddress(){
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
 * @brief Function to connect to the internet and process any network activity
 * 
 * @param param
 */
void network_task(void *param)
{ 
    // Before the loop can start, it is required that a stable network connection is needed!
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Wifi is trying to connect to the network: ");

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("");
    Serial.println("Wifi connected to network!");

    while (true) {

    }
}


