#ifndef TI_AS_BUFFERS_HPP
#define TI_AS_BUFFERS_HPP


#include <Arduino.h>


extern QueueHandle_t color_sensor_queue;
extern QueueHandle_t mqtt_data_queue;
extern QueueHandle_t tof_sensor_data_queue;
extern QueueHandle_t logger_queue;


#endif