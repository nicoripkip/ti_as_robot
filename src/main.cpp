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
#include <vector>
#include "arm.hpp"


struct TOFSensorData slam_tof_data[100];
struct robot_pos_t slam_pos_data[200]; 


TaskHandle_t motor_task_ptr;
TaskHandle_t network_task_ptr;
TaskHandle_t bluetooth_task_ptr;
TaskHandle_t tof_sensor_task_ptr;
TaskHandle_t camera_sensor_task_ptr;
TaskHandle_t magneto_sensor_task_ptr;
TaskHandle_t arm_task_ptr;


int stepss = 0;
int turning = 0;


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
  init_map();

    // Init global state object which holds the states of the robot
  global_object_state.semaphore = xSemaphoreCreateBinary();
  global_object_state.found_object = false;
  global_object_state.action = ACTION_IDLE;
  xSemaphoreGive(global_object_state.semaphore);

  // Register all tasks needed for the bot to work
  xTaskCreatePinnedToCore(motor_task, "Motor Task", MIN_TASK_STACK_SIZE, NULL, 1, &motor_task_ptr, CORE_NUM_2);
  xTaskCreatePinnedToCore(network_task, "Network Task", MIN_TASK_STACK_SIZE, NULL, 1, &network_task_ptr, CORE_NUM_2);
  xTaskCreatePinnedToCore(bluetooth_task, "Bluetooth Task", MIN_TASK_STACK_SIZE, NULL, 1, &bluetooth_task_ptr, CORE_NUM_2);
  xTaskCreatePinnedToCore(tof_sensor_task, "TOF Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &tof_sensor_task_ptr, CORE_NUM_1);
  xTaskCreatePinnedToCore(camera_sensor_task, "Camera Sensor Task", CAMERA_TASK_STACK_SIZE, NULL, 1, &camera_sensor_task_ptr, CORE_NUM_1);
  xTaskCreatePinnedToCore(magneto_sensor_task, "Magneto Sensor Task", MIN_TASK_STACK_SIZE, NULL, 1, &magneto_sensor_task_ptr, CORE_NUM_2);
  // xTaskCreatePinnedToCore(arm_task, "Arm Procedure Task", MIN_TASK_STACK_SIZE, NULL, 1, &arm_task_ptr, CORE_NUM_2);


  // Start motors
  motor1_data.i_run = true;
  motor2_data.i_run = true;      
  
  // Setup buffers
}


/**
 * @brief Function that controls the state of the RTOS
 * 
 */
void loop()
{
  uint16_t tsp = 0;
  uint16_t psp = 0;

  // Make sure all buffers are empty
  memset(slam_pos_data, 0, 100);
  memset(slam_tof_data, 0, 100);

  // update_motor_state(global_object_state.action);

  // Search path for the robot
  if (global_object_state.action == ACTION_SEARCHING) {
    if (stepss < 300) {
      motor1_data.i_forward = true;
      motor1_data.i_forward = true;

      motor1_data.i_backward = false;
      motor2_data.i_backward = false;

      stepss++;
    } else {
      motor1_data.i_turn_left = true;
      motor2_data.i_turn_left = true;

      motor1_data.i_forward = false;
      motor2_data.i_forward = false;

      turning++;
      if (turning >= 120) { 
        stepss = 0;
        turning = 0;
      }
    }
  }


  // Declaration of variables
  struct TOFSensorData tof_data;
  struct robot_pos_t  robot_data;


  if (tof_sensor_data_queue != nullptr) {
    // Get data from buffer
    while (xQueueReceive(tof_sensor_data_queue, &tof_data, 0)) {
      slam_tof_data[tsp] = tof_data;
      tsp++;
    }
  }

  if (robot_pos_queue != nullptr) {
    while (xQueueReceive(robot_pos_queue, &robot_data, 0)) {
      slam_pos_data[psp] = robot_data;
      psp++;
    }
  }


  // Steer robot when object is detected
  for (uint8_t i = 0; i < tsp; i++) {
    if (slam_tof_data[i].distance < 200 && slam_tof_data[i].degree <= 90) {
      motor1_data.i_turn_left = true;
      motor2_data.i_turn_left = true;

      motor1_data.i_turn_right = false;
      motor2_data.i_turn_right = false;

      motor1_data.i_forward = false;
      motor2_data.i_forward = false;

      stepss = 0;

      turning = 0;
      while (true) {
        turning++;
        if (turning >= 100) break;
      }
    } else if (slam_tof_data[i].distance < 200 && slam_tof_data[i].degree > 90) {
      motor1_data.i_turn_left = false;
      motor2_data.i_turn_left = false;

      motor1_data.i_turn_right = true;
      motor2_data.i_turn_right = true;

      motor1_data.i_forward = false;
      motor2_data.i_forward = false;

      stepss = 0;

      turning = 0;
      while (true) {
        turning++;
        if (turning >= 100) break;
      }
    }
  }


  if (mqtt_data_queue != nullptr) {
    char buffer[255];
    memset(buffer, 0, 255);

    snprintf(buffer, 255, "{ \"name\": \"%s\", \"coords\": [%0.2f, %0.2f], \"action\": \"%s\", \"network\": { \"online\": %d }, \"sensors\": { \"tof_sensor\": %d, \"magneto\": %d, \"servo\": %d } }", DEVICE_NAME, robot_data.pos.x_coord, robot_data.pos.y_coord, "searching", 1, tof_data.distance, robot_data.rotation, tof_data.degree);

    Serial.println(buffer);

    xQueueSend(mqtt_data_queue, buffer, 10);
  }

  // Update slam map
  update_map(slam_tof_data, slam_pos_data, tsp, psp);
  upload_map();

  // limit loop at 100000hz
  delayMicroseconds(100000);
}