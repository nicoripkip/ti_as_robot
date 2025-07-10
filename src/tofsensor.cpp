#include "tofsensor.hpp"
#include <Arduino.h>
#include "i2chandler.hpp"
#include "config.hpp"
#include "Adafruit_VL53L1X.h"
#include "buffers.hpp"


bool turn_left = false;
bool turn_right = false;
const uint32_t min_pwm_pulse_width = 500; // i guess miliseconds
const uint32_t max_pwm_pulse_width = 2500;
// VL53L0X lox;
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();


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

    //Start with the servo turning right
    turn_right = true;
    uint8_t tellen = 90;

    uint16_t MAX_TURN_RANGE = 0;
    if (SERVO_TOF_SENSOR_PWM_360)   MAX_TURN_RANGE = 360;
    else                            MAX_TURN_RANGE = 180;  

    // Reset servo on midpoint
    ledcWrite(SERVO_TOF_SENSOR_PWM_CHANNEL, turn_servo(tellen));
    // Init the TOF sensor
    bool err = false;
    while (!err) err = i2c_take_semaphore(1);

    if (!vl53.begin(TOF_SENSOR_ADDRESS, &Wire)) {
        Serial.println("Failed to intialize the sensor!");
    }
    Serial.println(F("VL53L1X sensor OK!"));

    if (! vl53.startRanging()) {
        Serial.print(F("Couldn't start ranging: "));
        Serial.println(vl53.vl_status);
        while (1)  delay(10);
    }

    Serial.println(F("Ranging started"));
    vl53.setTimingBudget(10);
    Serial.print(F("Timing budget (ms): "));
    Serial.println(vl53.getTimingBudget());

    i2c_give_semaphore(1);

    // Setup buffers
    tof_sensor_data_queue = xQueueCreate(100, sizeof(struct TOFSensorData));

    uint32_t ms;

    while (true) {
        struct TOFSensorData tof_data;
        bool err;

        err = i2c_take_semaphore(1);
        if (err) {

            tof_data.distance = vl53.distance();
            if (tof_data.distance >= 300) tof_data.distance = 65535;
            tof_data.degree = tellen;
            tof_data.scan_interval = micros();

            i2c_give_semaphore(1);
        } 

        if (tof_data.distance == -1) {
            // something went wrong!
            Serial.print(F("Couldn't get distance: "));
            Serial.println(vl53.vl_status);
        } 

        if (turn_left) {
            if ( tellen >= MAX_TURN_RANGE) {
                turn_left = false;
                turn_right = true;
            } else {
                tellen += 1;
            }
        } else if (turn_right) {
            if (tellen <= 0) {
                turn_left = true;
                turn_right = false;
            } else {
                tellen -= 1;
            }
        }

        ledcWrite(SERVO_TOF_SENSOR_PWM_CHANNEL, turn_servo(tellen));

        if (tof_sensor_data_queue != nullptr) {
            xQueueSend(tof_sensor_data_queue, &tof_data, 0);
        }

        delay(1);
    }
}