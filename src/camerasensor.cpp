#include "camerasensor.hpp"
#include <Arduino.h>
#include "config.hpp"
#include "buffers.hpp"
#include "network.hpp"


HardwareSerial ESP_CAM_SERIAL(2);
uint8_t buffer[CAMERA_BUFFER_IMAGE_SIZE];


void camera_sensor_task(void* param)
{
    ESP_CAM_SERIAL.begin(CAMERA_UART_BAUD_RATE, SERIAL_8N1, CAMERA_UART_RX_PIN, CAMERA_UART_TX_PIN);
    delay(100);

    while (true)
    {
        // Serial.println("Asking for image...");
        ESP_CAM_SERIAL.write('C');

        // Wait for header (blocking)
        while (ESP_CAM_SERIAL.available() < 4) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        if (ESP_CAM_SERIAL.read() == 0xA5 && ESP_CAM_SERIAL.read() == 0x5A) {
            uint16_t len;
            ESP_CAM_SERIAL.readBytes((char*)&len, 2);

            // Serial.printf("Receiving image of %u bytes\n", len);

            uint8_t* image = (uint8_t*)malloc(len);
            if (!image) {
                Serial.println("[error] Memory allocation failed");
                continue;  // Try next iteration 
            }

            size_t bytesRead = ESP_CAM_SERIAL.readBytes(image, len);
                
            
            if (bytesRead != len) {
                // Serial.printf("[error] Expected %u bytes, but got %u bytes\n", len, bytesRead);
                free(image);
                continue;
            }

            // ✅ Now the image buffer is valid and full
            // Serial.println("Image received successfully.");

            // Optional image processing here:
            // detect_and_draw_battery(image, IMG_WIDTH, IMG_HEIGHT, len);

            for (int i = 0; i < MAX_WS_CLIENTS; ++i) {
                if (wsConnectedFlags[i]) {
                    wsClients[i].sendBinary((const char*)image, len);
                    // Serial.printf("Image sent over WebSocket[%d].\n", i);
                } else {
                    // Serial.printf("WebSocket[%d] not connected - image not sent.\n", i);
                }
            }

            free(image);
        }
        else {
            Serial.println("[error] Invalid header received");
            while (ESP_CAM_SERIAL.available()) ESP_CAM_SERIAL.read(); // flush garbage
        }

        vTaskDelay(150 / portTICK_PERIOD_MS);  // small delay before next request
    }
}