#include "motor.hpp"
#include <Arduino.h>
#include "config.hpp"
#include "buffers.hpp"


hw_timer_t* timer;
volatile bool step_state = false;


/**
 * @brief Interrupt method to perform a step to move the stepper motors
 * 
 */
void IRAM_ATTR move_motor() 
{
    if (motor1_data.o_running && motor2_data.o_running) {
        if (step_state) {
            digitalWrite(MOTOR_LEFT_STEP_PIN, HIGH);
            digitalWrite(MOTOR_RIGHT_STEP_PIN, HIGH);
        } else {
            digitalWrite(MOTOR_LEFT_STEP_PIN, LOW);
            digitalWrite(MOTOR_RIGHT_STEP_PIN, LOW);
        }
    }

    step_state = !step_state;
}


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
    motor->change  = false;

    motor->semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(motor->semaphore);

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
    digitalWrite(MOTOR_RIGHT_DIRECTION_PIN, LOW);

    // init timer
    timer = timerBegin(0, 80, true);

    timerAttachInterrupt(timer, &move_motor, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

    // This while loop will controll all the motor stages
    while (true) {
        if (motor1_data.i_run && motor2_data.i_run) {
            motor1_data.o_running = true;
            motor2_data.o_running = true;

            BaseType_t err;

            if (motor1_data.i_forward && !motor1_data.i_backward && motor1_data.change) {
                err = xSemaphoreTake(motor1_data.semaphore, portMAX_DELAY);
                if (err != pdTRUE) continue;
                

                motor1_data.o_forward = true;
                motor1_data.o_backward = false;
                motor1_data.change = false;

                timerAlarmDisable(timer);

                digitalWrite(MOTOR_LEFT_DIRECTION_PIN, LOW);

                delayMicroseconds(100);

                timerAlarmEnable(timer);

                Serial.println("Motor 1 turning forward!");

                xSemaphoreGive(motor1_data.semaphore);
            } else if (!motor1_data.i_forward && motor1_data.i_backward && motor1_data.change) {
                err = xSemaphoreTake(motor1_data.semaphore, portMAX_DELAY);
                if (err != pdTRUE) continue;

                motor1_data.o_forward = false;
                motor1_data.o_backward = true;
                motor1_data.change = false;

                timerAlarmDisable(timer);

                digitalWrite(MOTOR_LEFT_DIRECTION_PIN, HIGH);

                delayMicroseconds(100);

                timerAlarmEnable(timer);

                Serial.println("Motor 1 turning backward!");

                xSemaphoreGive(motor1_data.semaphore);
            }

            if (motor2_data.i_forward && !motor2_data.i_backward && motor2_data.change) {
                err = xSemaphoreTake(motor2_data.semaphore, portMAX_DELAY);
                if (err != pdTRUE) continue;

                motor2_data.o_forward = true;
                motor2_data.o_backward = false;
                motor2_data.change = false;
                
                timerAlarmDisable(timer);

                digitalWrite(MOTOR_RIGHT_DIRECTION_PIN, LOW);
                
                delayMicroseconds(100);

                timerAlarmEnable(timer);

                Serial.println("Motor 2 turning forward!");

                xSemaphoreGive(motor2_data.semaphore);
            } else if (!motor2_data.i_forward && motor2_data.i_backward && motor2_data.change) {
                err = xSemaphoreTake(motor2_data.semaphore, portMAX_DELAY);
                if (err != pdTRUE) continue;

                motor2_data.o_forward = false;
                motor2_data.o_backward = true;
                motor2_data.change = false;

                timerAlarmDisable(timer);

                digitalWrite(MOTOR_RIGHT_DIRECTION_PIN, HIGH);
                
                delayMicroseconds(100);

                timerAlarmEnable(timer);

                Serial.println("Motor 2 turning backward!");

                xSemaphoreGive(motor2_data.semaphore);
            }
        }
    }
}