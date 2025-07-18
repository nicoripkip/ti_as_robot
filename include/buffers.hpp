#ifndef TI_AS_BUFFERS_HPP
#define TI_AS_BUFFERS_HPP


#include <Arduino.h>
#include "motor.hpp"
#include "utils.hpp"


extern QueueHandle_t            color_sensor_queue;
extern QueueHandle_t            mqtt_data_queue;
extern QueueHandle_t            tof_sensor_data_queue;
extern QueueHandle_t            magneto_sensor_data_queue;
extern QueueHandle_t            logger_queue;
extern QueueHandle_t            robot_pos_queue;
extern QueueHandle_t            mqtt_map_queue;
extern QueueHandle_t            scent_map_queue;

extern motor_data_t             motor1_data;
extern motor_data_t             motor2_data;

extern struct object_state_t    global_object_state;

extern uint16_t                 magneto_rotation;


#endif