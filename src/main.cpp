#include <Arduino.h>
#include "config.hpp"
#include "motor.hpp"
#include "network.hpp"
#include "bluetooth.hpp"
#include <WiFi.h>
#include "colorsensor.hpp"
#include "tofsensor.hpp"


TaskHandle_t motor_task_ptr;
TaskHandle_t network_task_ptr;
TaskHandle_t bluetooth_task_ptr;
TaskHandle_t tof_sensor_task_ptr;
TaskHandle_t color_sensor_task_ptr;


/**
 * @brief Setup basic tasks for the Microcontroller
 * 
 */
void setup()
{
  // Enable Serial debugging
  Serial.begin(DEVICE_BAUD_RATE);

  // Register all tasks needed for the bot to work
  xTaskCreatePinnedToCore(motor_task, "Motor Task", MIN_TASK_STACK_SIZE, NULL, 1, &motor_task_ptr, 1);
  xTaskCreatePinnedToCore(network_task, "Network Task", MIN_TASK_STACK_SIZE, NULL, 1, &network_task_ptr, 1);
  xTaskCreatePinnedToCore(bluetooth_task, "Bluetooth Task", MIN_TASK_STACK_SIZE, NULL, 1, &bluetooth_task_ptr, 1);
  xTaskCreatePinnedToCore(tof_sensor_task, "TOF Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &tof_sensor_task_ptr, 1);
  xTaskCreatePinnedToCore(color_sensor_task, "Color Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &color_sensor_task_ptr, 1);

  // Start scheduler
  vTaskStartScheduler();
}


/**
 * @brief Function that controls the state of the RTOS
 * 
 */
void loop()
{
  Serial.println("Dit print elke 3 seconde!");
  delay(3000);
}