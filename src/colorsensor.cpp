#include "colorsensor.hpp"
#include <Arduino.h>
#include "Adafruit_TCS34725.h"
#include "config.hpp"


Adafruit_TCS34725 color_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_4X);
bool color_sensor_enabled = false;


/**
 * @brief Function to start a thread with all the color sensor logic
 * 
 * @param param
 */
void color_sensor_task(void* param)
{
    if (color_sensor.begin()) {
        Serial.println("Color Sensor started!");
        color_sensor_enabled = true;
    }

    while (true) {
        if (color_sensor_enabled) {
            uint16_t r, g, b, c, colorTemp, lux;
            color_sensor.getRawData(&r, &g, &b, &c);
        }


    }
}