#include "display.hpp"
#include "config.hpp"
#include <Arduino.h>
#include "buffers.hpp"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>


const uint16_t SCREEN_WIDTH = 128;
const uint16_t SCREEN_HEIGHT = 64;


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);


/**
 * @brief Function to drive the displays
 * 
 * @param param
 */
void displayText(int x, int y, const String& text, int textSize = 1) {
    display.setTextSize(textSize);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x, y);
    display.println(text);
}

void display_task(void* param)
{
    bool errd = display.begin(SSD1306_SWITCHCAPVCC, OLED_DISPLAY_ADDRESS);
    if (!errd) {
        Serial.println("Can't initialize display!");
        vTaskDelete(NULL);
        return;
    }

    display.clearDisplay();
    displayText(0, 0, "Display Ready!");
    display.display();
    delay(2000);
    Serial.println("display ready!");

    while (true) {
        display.clearDisplay();

        // Example dynamic update
        displayText(0, 0, "Hello again!");
        display.display();

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


