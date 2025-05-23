#ifndef TI_AS_MOTOR_HPP
#define TI_AS_MOTOR_HPP


#include "Arduino.h"


typedef enum {
    MOTOR_ERR_OK = 0
} motor_err_t;


typedef enum {
    MICROSTEP_NONE  = 0,
    MICROSTEP_4X    = 1,
    MICROSTEP_16X   = 2
} microstep_conf_t;


typedef struct {
    char                    motor_name[12];

    // Technical info about the motor
    uint8_t                 gear_count;
    uint8_t                 rpm;
    bool                    microstepping_enabled;
    microstep_conf_t        conf;

    // States of the motor
    bool                    forward;
    bool                    backward;
    bool                    running;
    bool                    turning;
    bool                    reset;
    bool                    sleep;

    SemaphoreHandle_t       semaphore;
} motor_data_t;


void motor_task(void *param);


#endif