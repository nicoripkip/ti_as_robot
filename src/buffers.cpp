#include "buffers.hpp"
#include <Arduino.h>


/**
 * @brief Register all queues for the buffers
 * 
 */
QueueHandle_t color_sensor_queue;
QueueHandle_t mqtt_data_queue;
QueueHandle_t tof_sensor_data_queue;
QueueHandle_t magneto_sensor_data_queue;
QueueHandle_t logger_queue;
QueueHandle_t robot_pos_queue;
QueueHandle_t mqtt_map_queue;


/**
 * @brief Register here all motor data
 * 
 */
motor_data_t  motor1_data;
motor_data_t  motor2_data;


uint16_t magneto_rotation;
