#include <Arduino.h>
#include "config.hpp"
#include "motor.hpp"
#include "network.hpp"
#include "bluetooth.hpp"
#include <WiFi.h>
#include "colorsensor.hpp"
#include "tofsensor.hpp"
#include <Wire.h>
#include "buffers.hpp"


TaskHandle_t motor_task_ptr;
TaskHandle_t network_task_ptr;
TaskHandle_t bluetooth_task_ptr;
TaskHandle_t tof_sensor_task_ptr;
TaskHandle_t color_sensor_task_ptr;
TaskHandle_t magneto_sensor_task_ptr;


/**
 * @brief Setup basic tasks for the Microcontroller
 * 
 */
void setup()
{
  // Enable Serial debugging
  if (ENABLE_DEBUGGING) Serial.begin(DEVICE_BAUD_RATE);

  // Enable pins as i2c pins
  if (ENABLE_I2C_BUS_1) Wire.begin(I2C_BUS_1_SDA_PIN, I2C_SDL_1_SCL_PIN);
  if (ENABLE_I2C_BUS_2) Wire1.begin(I2C_BUS_2_SDA_PIN, I2C_BUS_2_SDL_PIN);

  // Register all tasks needed for the bot to work
  xTaskCreatePinnedToCore(motor_task, "Motor Task", MIN_TASK_STACK_SIZE, NULL, 1, &motor_task_ptr, 1);
  xTaskCreatePinnedToCore(network_task, "Network Task", MIN_TASK_STACK_SIZE, NULL, 1, &network_task_ptr, 1);
  xTaskCreatePinnedToCore(bluetooth_task, "Bluetooth Task", MIN_TASK_STACK_SIZE, NULL, 1, &bluetooth_task_ptr, 1);
  xTaskCreatePinnedToCore(tof_sensor_task, "TOF Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &tof_sensor_task_ptr, 1);
  xTaskCreatePinnedToCore(color_sensor_task, "Color Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &color_sensor_task_ptr, 1);

  // Start scheduler
  // vTaskStartScheduler();
}


/**
 * @brief Function that controls the state of the RTOS
 * 
 */
void loop()
{
  // Serial.println("Dit print elke 3 seconde!");
  delay(1000);

  color_data_t color_sensor_data;

  // if (uxQueueMessagesWaiting(color_sensor_queue) > 0) {
  //   // TODO: process all incoming data from the different sensors
  //   xQueueReceive(color_sensor_queue, &color_sensor_data, portMAX_DELAY);

  //   Serial.print("R: ");
  //   Serial.print(color_sensor_data.rgb[0]);
  //   Serial.print(", G: ");
  //   Serial.print(color_sensor_data.rgb[1]);
  //   Serial.print(", B: ");
  //   Serial.println(color_sensor_data.rgb[2]);

  //   char buffer[256];
  //   memset(buffer, 0, 256);

  //   snprintf(buffer, 256, "{ \"des\": \"robot2hmi\", \"r\": %d, \"g\": %d, \"b\": %d }");

  //   xQueueSend(mqtt_data_queue, &buffer, portMAX_DELAY);
  // }
}