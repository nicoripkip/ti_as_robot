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
    ESP_CAM_SERIAL.begin(CAMERA_UART_BAUD_RATE, SERIAL_8N1, CAMERA_UART_RX_PIN, CAMERA_UART_TX_PIN);
     
    // Clean image buffer
    memset(buffer, 0, CAMERA_BUFFER_IMAGE_SIZE);

    while (true) {
        if (ESP_CAM_SERIAL.available()) {

        }
    }
}