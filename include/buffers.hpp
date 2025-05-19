#ifndef TI_AS_BUFFERS_HPP
#define TI_AS_BUFFERS_HPP


#include <Arduino.h>


extern QueueHandle_t color_sensor_queue;
extern SemaphoreHandle_t color_sensor_queue_semaphore;

extern QueueHandle_t mqtt_data_queue;
extern SemaphoreHandle_t mqtt_data_queue_semaphore;


#endif