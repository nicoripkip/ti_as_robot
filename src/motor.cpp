#include "motor.hpp"


#include <Arduino.h>


/**
 * @brief This function contains all the logic to make the motors run
 * 
 * @param param
 */
void motor_task(void *param)
{
    while (true) {
        Serial.println("Motortje draait!");
        delay(1000);
    }
}