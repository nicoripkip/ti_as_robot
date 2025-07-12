#pragma once


#ifndef TI_AS_CONFIG_HPP
#define TI_AS_CONFIG_HPP


#include <cstdint>


/**
 * @brief General settings for the device goes here
 * 
 */
const char DEVICE_NAME[]                    = "bot1";
const uint32_t DEVICE_BAUD_RATE             = 115200;


/**
 * @brief Here goes all the settings for configuring the parameters of FreeRTOS
 *  
 */
const uint32_t MIN_TASK_STACK_SIZE          = 10000;
const uint32_t CAMERA_TASK_STACK_SIZE       = 30000;
const uint8_t CORE_NUM_1                    = 0;
const uint8_t CORE_NUM_2                    = 1;


/**
 * @brief Here goes all the settings for networking and wireless data transmission
 * 
 */
// const char WIFI_SSID[]                      = "iotroam";
// const char WIFI_PASSWORD[]                  = "henkdepotvis";
// const char WIFI_SSID[]                  = "Ziggo3876364";
// const char WIFI_PASSWORD[]              = "otprnkY5ows7kucn";
// const char WIFI_SSID[]                      = "H369A432F1F";
// const char WIFI_PASSWORD[]                  = "3D6269C97CEA";
const char WIFI_SSID[]                      = "ti_as_robot_hotspot";
const char WIFI_PASSWORD[]                  = "KankerIOTroam";


/**
 * @brief Settings for connecting with MQTT
 * 
 */
const char MQTT_CLIENT_ID[]                 = "robot_xxxx";
const bool MQTT_CLIENT_ANONYMOUS            = true;
const char MQTT_USER[]                      = "";
const char MQTT_PASS[]                      = "";
const char MQTT_SERVER[]                    = "145.24.223.53";
const int MQTT_PORT                         = 9000;
const uint8_t MQTT_MAX_PACk_SIZE            = 128;


/**
 * @brief Settings for the websocket
 * 
 */
const char WS_SERVER_HOST[]                 = "145.24.223.53";
const uint16_t WS_SERVER_PORT               = 8080;


/**
 * @brief I/O configuration
 * 
 */
const uint8_t MOTOR_LEFT_STEP_PIN           = 5;
const uint8_t MOTOR_LEFT_DIRECTION_PIN      = 6;
const uint8_t MOTOR_LEFT_SLEEP_PIN          = 4;
const uint8_t MOTOR_LEFT_RESET_PIN          = 0;

const uint8_t MOTOR_RIGHT_STEP_PIN          = 16;
const uint8_t MOTOR_RIGHT_DIRECTION_PIN     = 17;
const uint8_t MOTOR_RIGHT_SLEEP_PIN         = 15;
const uint8_t MOTOR_RIGHT_RESET_PIN         = 0;

const bool ENABLE_I2C_BUS_1                 = true;
const uint8_t I2C_BUS_1_SDA_PIN             = 13;
const uint8_t I2C_SDL_1_SCL_PIN             = 12;

const bool ENABLE_I2C_BUS_2                 = false;
const uint8_t I2C_BUS_2_SDA_PIN             = 0;
const uint8_t I2C_BUS_2_SDL_PIN             = 0;

const uint8_t TOF_SENSOR_ADDRESS            = 0x29;
const uint16_t MAGNETO_SENSOR_ADDRESS       = 0x0D;
const uint16_t OLED_DISPLAY_ADDRESS         = 0x3C;

const uint64_t CAMERA_UART_BAUD_RATE        = 4000000;
const uint8_t CAMERA_UART_RX_PIN            = 35;
const uint8_t CAMERA_UART_TX_PIN            = 36;
const uint32_t CAMERA_BUFFER_IMAGE_SIZE     = 15000;

const bool SERVO_TOF_SENSOR_PWM_360         = false;
const uint8_t SERVO_TOF_SENSOR_PWM_PIN      = 11;
const uint8_t SERVO_TOF_SENSOR_PWM_CHANNEL  = 0;
const uint16_t SERVO_TOF_SENSOR_PWM_FREQ    = 50;
const uint16_t SERVO_TOF_SENSOR_PWM_RES     = 14;

const uint8_t BTN_PIN_1                     = 0;
const uint8_t BTN_PIN_2                     = 0;


/**
 * @brief Debugging settings
 * 
 */
const bool ENABLE_DEBUGGING                 = true;
const bool ENABLE_NETWORK_LOGGING           = false;


#endif