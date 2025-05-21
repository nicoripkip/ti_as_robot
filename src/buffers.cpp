#include "buffers.hpp"
#include <Arduino.h>


/**
 * @brief Register all queues for the buffers
 * 
 */
QueueHandle_t color_sensor_queue;
QueueHandle_t mqtt_data_queue;
QueueHandle_t tof_sensor_data_queue;
