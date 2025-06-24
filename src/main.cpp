#include <Arduino.h>
#include "config.hpp"
#include "motor.hpp"
#include "network.hpp"
#include "bluetooth.hpp"
#include <WiFi.h>
#include "camerasensor.hpp"
#include "tofsensor.hpp"
#include <Wire.h>
#include "buffers.hpp"
#include "slam.hpp"
#include "i2chandler.hpp"
#include "magnetosensor.hpp"


TaskHandle_t motor_task_ptr;
TaskHandle_t network_task_ptr;
TaskHandle_t bluetooth_task_ptr;
TaskHandle_t tof_sensor_task_ptr;
TaskHandle_t camera_sensor_task_ptr;
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
  if (ENABLE_I2C_BUS_1) i2c_init(1, I2C_BUS_1_SDA_PIN, I2C_SDL_1_SCL_PIN);
  if (ENABLE_I2C_BUS_2) i2c_init(2, I2C_BUS_2_SDA_PIN, I2C_BUS_2_SDL_PIN);

  // Init slam
  // init_map();

  // Register all tasks needed for the bot to work
  xTaskCreatePinnedToCore(motor_task, "Motor Task", MIN_TASK_STACK_SIZE, NULL, 1, &motor_task_ptr, 1);
  xTaskCreatePinnedToCore(network_task, "Network Task", MIN_TASK_STACK_SIZE, NULL, 1, &network_task_ptr, 1);
  xTaskCreatePinnedToCore(bluetooth_task, "Bluetooth Task", MIN_TASK_STACK_SIZE, NULL, 1, &bluetooth_task_ptr, 1);
  xTaskCreatePinnedToCore(tof_sensor_task, "TOF Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &tof_sensor_task_ptr, 1);
  xTaskCreatePinnedToCore(camera_sensor_task, "Camera Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &camera_sensor_task_ptr, 1);
  xTaskCreatePinnedToCore(magneto_sensor_task, "Magneto Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &magneto_sensor_task_ptr, 1);

  // Start scheduler
  // vTaskStartScheduler();

  motor1_data.i_run = true;
  motor2_data.i_run = true;         
}


/**
 * @brief Function that controls the state of the RTOS
 * 
 */
void loop()
{
  if (Serial.available()) {
    char n = Serial.read();
    
    if (n == 'f') {
      xSemaphoreTake(motor1_data.semaphore, portMAX_DELAY);
      xSemaphoreTake(motor2_data.semaphore, portMAX_DELAY);

      motor1_data.i_forward = true;
      motor1_data.i_backward = false;

      motor2_data.i_forward = true;
      motor2_data.i_backward = false;

      motor1_data.change = true;
      motor2_data.change = true;

      xSemaphoreGive(motor1_data.semaphore);
      xSemaphoreGive(motor2_data.semaphore);
    } else if ('b') {
      xSemaphoreTake(motor1_data.semaphore, portMAX_DELAY);
      xSemaphoreTake(motor2_data.semaphore, portMAX_DELAY);

      motor1_data.i_forward = false;
      motor1_data.i_backward = true;

      motor2_data.i_forward = false;
      motor2_data.i_backward = true; 

      motor1_data.change = true;
      motor2_data.change = true;

      xSemaphoreGive(motor1_data.semaphore);
      xSemaphoreGive(motor2_data.semaphore);
    }
  }


  // Declaration of variables
  TOFSensorData tof_data = { 0, 0 };
  MagnetoSensorData magneto_data = {0, 0, 0};

  // Serial.println("Dit print elke 3 seconde!");

  if (tof_sensor_data_queue != nullptr) {
    // Get data from buffer
    xQueueReceive(tof_sensor_data_queue, &tof_data, 50);

    // Serial.print("Distance: ");
    // Serial.println(tof_data.distance);
  }

  if (magneto_sensor_data_queue != nullptr) {
    xQueueReceive(magneto_sensor_data_queue, &magneto_data, 50);
  }

  if (mqtt_data_queue != nullptr) {
    char buffer[256];
    memset(buffer, 0, 256);

    snprintf(buffer, 256, "{ \"name\": \"%s\", \"role\": \"%s\", \"coords\": [%d, %d], \"action\": \"%s\", \"network\": { \"online\": %d }, \"sensors\": { \"tof_sensor\": %d, \"magneto\": %d } }", DEVICE_NAME, "", 3, 3, "", 1, tof_data.distance, magneto_data.degree);
    // snprintf(buffer, 256, "Distance: %d mm, Rotation: %d degrees", tof_data.distance, tof_data.degree);
    // snprintf(buffer, 256, "HAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");

    xQueueSend(mqtt_data_queue, buffer, 10);
  }

  // Serial.println("test");
}