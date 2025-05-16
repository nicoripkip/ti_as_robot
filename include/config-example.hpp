#ifndef TI_AS_CONFIG_HPP
#define TI_AS_CONFIG_HPP


#include <cstdint>


/**
 * @brief General settings for the device goes here
 * 
 */
const char DEVICE_NAME[]                = "robot_xxxx";


/**
 * @param Here goes all the settings for configuring the parameters of FreeRTOS
 *  
 */
const uint32_t MIN_TASK_STACK_SIZE      = 10000;


/**
 * @brief Here goes all the settings for networking and wireless data transmission
 * 
 */
const char WIFI_SSID[]                  = "<Your wifi SSID>";
const char WIFI_PASSWORD[]              = "<Your wifi password>";


/**
 * @brief Settings for connecting with MQTT
 * 
 */
const char MQTT_CLIENT_ID[]             = "robot_xxxx";
const bool MQTT_CLIENT_ANONYMOUS        = true;
const char MQTT_USER[]                  = "<MQTT username here>";
const char MQTT_PASS[]                  = "<MQTT password here>";
const char MQTT_SERVER[]                = "<MQTT server address here>";
const int MQTT_PORT                     = 1883;


/**
 * @brief I/O configuration
 * 
 */
const uint8_t motor_left_pins[4]        = {};
const uint8_t motor_right_pins[4]       = {};
const uint8_t color_sensor_pins[]       = {};
const uint8_t tof_sensor_pins[]         = {};
const uint8_t magneto_sensor_pins[]     = {};


#endif