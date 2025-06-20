#include "camerasensor.hpp"
#include <Arduino.h>
#include "config.hpp"
#include "buffers.hpp"


HardwareSerial ESP_CAM_SERIAL(2);
byte buffer[CAMERA_BUFFER_IMAGE_SIZE];


/**
 * @brief Function to start a thread with all the color sensor logic
 *  
 * @param param
 */
void camera_sensor_task(void* param)
{
    // Initialize the uart object
    ESP_CAM_SERIAL.begin(4000000, SERIAL_8N1, CAMERA_UART_RX_PIN, CAMERA_UART_TX_PIN);
     
    // Clean image buffer
    memset(buffer, 0, CAMERA_BUFFER_IMAGE_SIZE);

    while (true) {
        // Send 67 to ask for image
        Serial.println("Try to ask for an image");
        ESP_CAM_SERIAL.write('C');

        // Try to read received data from camera
        if (ESP_CAM_SERIAL.available()) {
            // Read data from the camera
            if (ESP_CAM_SERIAL.read() == 0xA5 && ESP_CAM_SERIAL.read() == 0x54) {
                // Reset buffer
                memset(buffer, 0, CAMERA_BUFFER_IMAGE_SIZE);

                uint16_t len;
                ESP_CAM_SERIAL.readBytes((char *)&len, 2);
                
                // Receive data from the image
                ESP_CAM_SERIAL.readBytes((char *)buffer, CAMERA_BUFFER_IMAGE_SIZE);

                Serial.print("Size of received image: ");
                Serial.println(len);   
            } else {
                Serial.println("[error] Not the correct header!");
                while (ESP_CAM_SERIAL.available()) {
                    uint8_t c = ESP_CAM_SERIAL.read();
                    Serial.print("bytes: ");
                    Serial.println(c, HEX);
                } 
            }
        };



        delay(100);
    }
}