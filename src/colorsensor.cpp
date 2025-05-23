#include "colorsensor.hpp"
#include <Arduino.h>
#include "Adafruit_TCS34725.h"
#include "config.hpp"
#include "buffers.hpp"


Adafruit_TCS34725 color_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_4X);
bool color_sensor_enabled = false;


/**
 * @brief Function to start a thread with all the color sensor logic
 *  
 * @param param
 */
void color_sensor_task(void* param)
{
    // Create a i2c object
    if (color_sensor.begin()) {
        Serial.println("Color Sensor started!");
        color_sensor_enabled = true;

        // Setup buffer for collecting data from the color sensor
        color_sensor_queue = xQueueCreate(10, sizeof(color_data_t));
    }

    while (true) {
        if (color_sensor_enabled) {
            color_data_t* sensor_data = (color_data_t*)malloc(sizeof(color_data_t));
            if (sensor_data == nullptr) {
                continue;
            }

            uint16_t r, g, b, c, colorTemp, lux;
            color_sensor.getRawData(&r, &g, &b, &c);

            sensor_data->rgb[0] = r;
            sensor_data->rgb[1] = g;
            sensor_data->rgb[2] = b;
            sensor_data->lux = color_sensor.calculateLux(r, g, b);
            sensor_data->temp = color_sensor.calculateColorTemperature_dn40(r, g, b, c);


            // Push the received data into the queue for further processing
            // if (color_sensor_queue != nullptr) xQueueSend(color_sensor_queue, sensor_data, portMAX_DELAY);
        }
    }
}