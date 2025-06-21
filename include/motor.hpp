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


/**
 * @brief Struct for handling all the motor data
 * 
 */
typedef struct {
    char                    motor_name[12];

    // Technical info about the motor
    uint8_t                 gear_count;
    uint8_t                 rpm;
    bool                    microstepping_enabled;
    microstep_conf_t        conf;

    // States of the motor inputs
    bool                    i_forward;
    bool                    i_backward;
    bool                    i_turn_left;
    bool                    i_turn_right;
    bool                    i_reset;
    bool                    i_sleep;
    bool                    i_run;

    // States voor de motor outputs
    volatile bool           o_forward;
    volatile bool           o_backward;
    volatile bool           o_running;
    volatile bool           o_turning;
    volatile bool           o_reset;
    volatile bool           o_sleep;

    // Handle synchronization across threads
    SemaphoreHandle_t       semaphore;
} motor_data_t;


void motor_task(void *param);


#endif