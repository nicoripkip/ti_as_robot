#include "motor.hpp"
#include <Arduino.h>
#include "config.hpp"


/**
 * @brief This function contains all the logic to make the motors run
 * 
 * @param param
 */
void motor_task(void *param)
{
    while (true) {
        Serial.println("Motortje draait!");
        delayMicroseconds(1000);
        digitalWrite(MOTOR_LEFT_STEP_PIN, HIGH);
        digitalWrite(MOTOR_RIGHT_STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(MOTOR_LEFT_STEP_PIN, LOW);
        digitalWrite(MOTOR_RIGHT_STEP_PIN, LOW);
    }
}