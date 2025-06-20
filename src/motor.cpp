#include "motor.hpp"
#include <Arduino.h>
#include "config.hpp"
#include "buffers.hpp"


/**
 * @brief Internal function for initializing data into the motor data structs
 * 
 * @param motor
 * @param name
 * @param gears
 * @param rpm 
 * @param ems Parameter for enabling microstepping in the motor or not
 * 
 * @return motor_err_t
 */
static motor_err_t init_motor_data(motor_data_t *motor, char *name,  uint8_t gears, uint32_t rpm, bool ems)
{
    memset(motor->motor_name, 0, 12);
    memcpy(motor->motor_name, name, 12);

    motor->gear_count = gears;
    motor->rpm = rpm;
    motor->microstepping_enabled = ems;

    if (motor->microstepping_enabled) motor->conf = MICROSTEP_4X;
    else motor->conf = MICROSTEP_NONE; 

    motor->o_forward = false;
    motor->o_backward = false;
    motor->o_running = false;
    motor->o_turning = false;
    motor->o_reset = false;
    motor->o_sleep = false;

    motor->semaphore = xSemaphoreCreateBinary();

    return MOTOR_ERR_OK;
}


/**
 * @brief This function contains all the logic to make the motors run
 * 
 * @param param
 */
void motor_task(void *param)
{
    // Configure pins for the motor
    pinMode(MOTOR_LEFT_STEP_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_SLEEP_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_RESET_PIN, OUTPUT);

    pinMode(MOTOR_RIGHT_STEP_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_SLEEP_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_RESET_PIN, OUTPUT);

    // Init motor buffers
    init_motor_data(&motor1_data, "motor_left", 64, 1000, false);
    init_motor_data(&motor2_data, "motor_right", 64, 1000, false);

    // Enable sleep pins to make sure
    digitalWrite(MOTOR_LEFT_SLEEP_PIN, HIGH);
    digitalWrite(MOTOR_RIGHT_SLEEP_PIN, HIGH);


    // Set direction pin
    digitalWrite(MOTOR_LEFT_DIRECTION_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_DIRECTION_PIN, HIGH);

    while (true) {
        // Serial.println("Motortje draait!");
        delayMicroseconds(500);
        digitalWrite(MOTOR_LEFT_STEP_PIN, HIGH);
        digitalWrite(MOTOR_RIGHT_STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(MOTOR_LEFT_STEP_PIN, LOW);
        digitalWrite(MOTOR_RIGHT_STEP_PIN, LOW);
    }
}