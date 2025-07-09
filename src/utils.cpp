#include "utils.hpp"
#include <Arduino.h>
#include "config.hpp"


const uint32_t min_pwm_pulse_width = 500; // i guess miliseconds
const uint32_t max_pwm_pulse_width = 2500;


/**
 * @brief Function that takes a angle in degrees as input and converts that to a dutycycle for a pwm channel
 * 
 * @param angle
 * 
 * @return uint32_t
 */
uint32_t calculate_duty_cycle(uint16_t angle) {
    int width = map(angle, 0, 180, min_pwm_pulse_width, max_pwm_pulse_width);

    uint32_t duty_cycle = width * ((1 << SERVO_TOF_SENSOR_PWM_RES) - 1) / 20000;

    return duty_cycle;
}
