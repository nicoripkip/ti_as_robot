#include "tofsensor.hpp"
#include <Arduino.h>
#include "config.hpp"


void tof_sensor_task(void* param)
{
    ledcSetup(SERVO_TOF_SENSOR_PWM_CHANNEL, SERVO_TOF_SENSOR_PWM_FREQ, SERVO_TOF_SENSOR_PWM_RES);

    

    while (true) {
        
    }
}