#include "buffers.hpp"
#include <Arduino.h>


/**
 * @brief Register all semaphores for the buffers
 * 
 */
QueueHandle_t color_sensor_queue;
QueueHandle_t mqtt_data_queue;


/**
 * @brief Register all interprocessor buffers
 * 
 */
SemaphoreHandle_t color_sensor_queue_semaphore;
SemaphoreHandle_t mqtt_data_queue_semaphore;