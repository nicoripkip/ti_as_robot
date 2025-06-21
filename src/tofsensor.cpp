#include "tofsensor.hpp"
#include <Arduino.h>
#include "i2chandler.hpp"
#include "config.hpp"


bool turn_left = false;
bool turn_right = false;


const uint32_t min_pwm_pulse_width = 500; // i guess miliseconds
const uint32_t max_pwm_pulse_width = 2500;


/**
 * @brief Function that takes a angle in degrees as input and converts that to a dutycycle for a pwm channel
 * 
 * @param angle
 * 
 * @return uint32_t
 */
uint32_t turn_servo(uint16_t angle) {
    int width = map(angle, 0, 180, min_pwm_pulse_width, max_pwm_pulse_width);

    uint32_t duty_cycle = width * ((1 << SERVO_TOF_SENSOR_PWM_RES) - 1) / 20000;

    return duty_cycle;
}


/**
 * @brief Function
 * 
 * @param param
 */
void tof_sensor_task(void* param)
{
    // Setup pwm channel to control the motor
    ledcSetup(SERVO_TOF_SENSOR_PWM_CHANNEL, SERVO_TOF_SENSOR_PWM_FREQ, SERVO_TOF_SENSOR_PWM_RES);
    ledcAttachPin(SERVO_TOF_SENSOR_PWM_PIN, SERVO_TOF_SENSOR_PWM_CHANNEL);


    // pinMode(35, OUTPUT);
    // digitalWrite(35, 1);

    //Start with the servo turning right
    turn_right = true;

    uint8_t tellen = 90;

    uint16_t MAX_TURN_RANGE = 0;
    if (SERVO_TOF_SENSOR_PWM_360)   MAX_TURN_RANGE = 360;
    else                            MAX_TURN_RANGE = 180;  

    // Reset servo on midpoint
    ledcWrite(SERVO_TOF_SENSOR_PWM_CHANNEL, turn_servo(tellen));

    byte data = i2c_read_byte(&Wire, 1, TOF_SENSOR_ADDRESS, 0xC0);
    Serial.print("TOF Sensor data read: ");
    Serial.println(data, HEX);

    while (true) {
        if (turn_left) {
            if ( tellen >= MAX_TURN_RANGE) {
                turn_left = false;
                turn_right = true;
            } else {
                tellen++;
            }
        }

        if (turn_right) {
            if (tellen <= 0) {
                turn_left = true;
                turn_right = false;
            } else {
                tellen--;
            }
        }

        ledcWrite(SERVO_TOF_SENSOR_PWM_CHANNEL, turn_servo(tellen));
        delay(1);
    }
}