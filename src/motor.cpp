#include "motor.hpp"
#include <Arduino.h>
#include "config.hpp"
#include "buffers.hpp"
#include "slam.hpp"


hw_timer_t* timer;
volatile bool step_state = false;
volatile uint32_t steps = 0;
volatile bool update_coord = false;


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

        // Flag when a step is taken and the coord can be logged of the robot
        if (!update_coord) update_coord = true;

        // Step the motor forward only when the motor is turning forward
        if (!motor1_data.o_turning || !motor2_data.o_running) steps++;
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
 * @brief Function that will set the motor to move forward
 * 
 * @return bool
 */
bool move_motor_forward()
{
    BaseType_t err1;
    BaseType_t err2;

    err1 = xSemaphoreTake(motor1_data.semaphore, portMAX_DELAY);
    err2 = xSemaphoreTake(motor2_data.semaphore, portMAX_DELAY);
    if (err1 != pdTRUE) return false;
    if (err2 != pdTRUE) {
        xSemaphoreGive(motor1_data.semaphore);
        return false;
    }

    motor1_data.o_forward = true;
    motor2_data.o_forward = true;

    motor1_data.o_backward = false;
    motor2_data.o_forward = false;

    motor1_data.o_turning = false;
    motor2_data.o_turning = false;

    digitalWrite(MOTOR_LEFT_DIRECTION_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_DIRECTION_PIN, LOW);

    xSemaphoreGive(motor1_data.semaphore);
    xSemaphoreGive(motor2_data.semaphore);

    return true;
}


/**
 * @brief 
 * 
 */
bool move_motor_backward() 
{
    BaseType_t err1;
    BaseType_t err2;

    err1 = xSemaphoreTake(motor1_data.semaphore, portMAX_DELAY);
    err2 = xSemaphoreTake(motor2_data.semaphore, portMAX_DELAY);
    if (err1 != pdTRUE) return false;
    if (err2 != pdTRUE) {
        xSemaphoreGive(motor1_data.semaphore);
        return false;
    }

    motor1_data.o_forward = false;
    motor2_data.o_forward = false;

    motor1_data.o_backward = true;
    motor2_data.o_forward = true;

    motor1_data.o_turning = false;
    motor2_data.o_turning = false;

    digitalWrite(MOTOR_LEFT_DIRECTION_PIN, HIGH);
    digitalWrite(MOTOR_RIGHT_DIRECTION_PIN, HIGH);

    xSemaphoreGive(motor1_data.semaphore);
    xSemaphoreGive(motor2_data.semaphore);

    return true;
}


bool move_motor_left()
{
    BaseType_t err1;
    BaseType_t err2;

    err1 = xSemaphoreTake(motor1_data.semaphore, portMAX_DELAY);
    err2 = xSemaphoreTake(motor2_data.semaphore, portMAX_DELAY);
    if (err1 != pdTRUE) return false;
    if (err2 != pdTRUE) {
        xSemaphoreGive(motor1_data.semaphore);
        return false;
    }

    motor1_data.o_forward = false;
    motor2_data.o_forward = false;

    motor1_data.o_backward = false;
    motor2_data.o_forward = false;

    motor1_data.o_turning = true;
    motor2_data.o_turning = true;

    digitalWrite(MOTOR_LEFT_DIRECTION_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_DIRECTION_PIN, HIGH);


    xSemaphoreGive(motor1_data.semaphore);
    xSemaphoreGive(motor2_data.semaphore);

    return true;
}


bool move_motor_right()
{
    BaseType_t err1;
    BaseType_t err2;

    err1 = xSemaphoreTake(motor1_data.semaphore, portMAX_DELAY);
    err2 = xSemaphoreTake(motor2_data.semaphore, portMAX_DELAY);
    if (err1 != pdTRUE) return false;
    if (err2 != pdTRUE) {
        xSemaphoreGive(motor1_data.semaphore);
        return false;
    }

    motor1_data.o_forward = false;
    motor2_data.o_forward = false;

    motor1_data.o_backward = false;
    motor2_data.o_forward = false;

    motor1_data.o_turning = true;
    motor2_data.o_turning = true;

    digitalWrite(MOTOR_LEFT_DIRECTION_PIN, HIGH);
    digitalWrite(MOTOR_RIGHT_DIRECTION_PIN, LOW);


    xSemaphoreGive(motor1_data.semaphore);
    xSemaphoreGive(motor2_data.semaphore);

    return true;
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

    // Init motor pos buffer
    robot_pos_queue = xQueueCreate(100, sizeof(struct robot_pos_t));

    // init timer
    timer = timerBegin(0, 80, true);

    timerAttachInterrupt(timer, &move_motor, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

    // This while loop will controll all the motor stages
    while (true) { 
        struct robot_pos_t robot_pos;

        // Process the steps of the robot to update the coordinate
        if (update_coord) {
            update_coord = false;
            robot_pos = update_robot_coord(steps, magneto_rotation);
            steps = 0;

            if (robot_pos_queue != nullptr) {
                xQueueSend(robot_pos_queue, &robot_pos, 10);
            }
        }

        // Do some control over the direction of the robot
        if (motor1_data.i_run && motor2_data.i_run) {
            motor1_data.o_running = true;
            motor2_data.o_running = true;

            if (motor1_data.i_forward && motor2_data.i_forward) {
                move_motor_forward();
            }

            if (motor1_data.i_turn_left && motor2_data.i_turn_left) {
                move_motor_left();
            }

            if (motor1_data.i_turn_right && motor2_data.i_turn_right) {
                move_motor_right();
            }
        }
    }
}