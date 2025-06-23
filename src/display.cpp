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
void display_task(void* param)
{
    // Configure these btns with their respective pulldown resistors so i dont have to care about these
    // Pin 1 is for navigating the options on the display
    // Pin 2 is for pressing ok to select an option on the display
    pinMode(BTN_PIN_1, INPUT_PULLDOWN);
    pinMode(BTN_PIN_2, INPUT_PULLDOWN);

    // Initialize display for configuration
    bool errd = display.begin(SSD1306_SWITCHCAPVCC, OLED_DISPLAY_ADDRESS);
    if (!errd) Serial.println("Can't initialize display!");

    // Wait 2 seconds for the display to initialize
    delay(2000);

    while (true) {
        display.clearDisplay();
    }
}