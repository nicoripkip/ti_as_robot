#ifndef TI_AS_BUFFERS_HPP
#define TI_AS_BUFFERS_HPP


#include <Arduino.h>
#include "motor.hpp"


extern QueueHandle_t    color_sensor_queue;
extern QueueHandle_t    mqtt_data_queue;
extern QueueHandle_t    tof_sensor_data_queue;
extern QueueHandle_t    logger_queue;

extern motor_data_t     motor1_data;
extern motor_data_t     motor2_data;


#endif