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
  global_object_state.marked_object = false;
  global_object_state.action = ACTION_IDLE;
  xSemaphoreGive(global_object_state.semaphore);

  // Register all tasks needed for the bot to work
  xTaskCreatePinnedToCore(motor_task, "Motor Task", MIN_TASK_STACK_SIZE, NULL, 1, &motor_task_ptr, CORE_NUM_2);
  xTaskCreatePinnedToCore(network_task, "Network Task", MIN_TASK_STACK_SIZE, NULL, 1, &network_task_ptr, CORE_NUM_1);
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
  struct TOFSensorData tof_data;
  struct robot_pos_t  robot_data;
  static bool turning_left = false;
  static bool turning_right = false;

  // Make sure all buffers are empty
  memset(slam_pos_data, 0, sizeof(slam_pos_data));
  memset(slam_tof_data, 0, sizeof(slam_tof_data));

  // update_motor_state(global_object_state.action);

  // Search path for the robot
  switch (global_object_state.action) {
    case ACTION_IDLE:
 
      motor1_data.i_run = false;
      motor2_data.i_run = false;  

      motor1_data.i_forward = false;
      motor2_data.i_forward = false;

      motor1_data.i_turn_left = false;
      motor2_data.i_turn_left = false;

      motor1_data.i_turn_right = false;
      motor2_data.i_turn_right = false;

      motor1_data.i_backward = false;
      motor2_data.i_backward = false;

      break;

    case ACTION_SEARCHING:

      motor1_data.i_run = true;
      motor2_data.i_run = true;  
      
      if (!turning_left && !turning_right) {
        motor1_data.i_forward = true;
        motor2_data.i_forward = true;

        motor1_data.i_turn_left = false;
        motor2_data.i_turn_left = false;

        motor1_data.i_turn_right = false;
        motor2_data.i_turn_right = false;

        motor1_data.i_backward = false;
        motor2_data.i_backward = false;
      }

      // if (detect_pheromone(&robot_data)) {
      //   BaseType_t err = xSemaphoreTake(global_object_state.semaphore, 10);
      //   while (err != pdTRUE) err = xSemaphoreTake(global_object_state.semaphore, 10);

      //   global_object_state.action = ACTION_FOLLOWING;

      //   xSemaphoreGive(global_object_state.semaphore);
      // }

      break;
    case ACTION_RETRIEVING:

      motor1_data.i_run = true;
      motor2_data.i_run = true; 

      if (global_object_state.found_object && !global_object_state.marked_object) {
        global_object_state.marked_pos = robot_data;
        global_object_state.marked_object = true;
      }
      
      // deposit_pheromone(&robot_data);



      break;

    case ACTION_FOLLOWING:

      break;
  }



  if (tof_sensor_data_queue != nullptr) {
    // Get data from buffer
    while (xQueueReceive(tof_sensor_data_queue, &tof_data, 0)) {
      slam_tof_data[tsp] = tof_data;

      tsp++;
    }
  }

  if (robot_pos_queue != nullptr) {
    while (xQueueReceive(robot_pos_queue, &robot_data, 1)) {
      slam_pos_data[psp] = robot_data;
      psp++;
    }
  }


  // Steer robot when object is detected
  static unsigned long turn_start_time = 0;


  if (turning_left || turning_right) {
    if (millis() - turn_start_time >= 2000) { // 500ms turn
      motor1_data.i_forward = true;
      motor2_data.i_forward = true;

      // Stop turning
      motor1_data.i_turn_left = false;
      motor2_data.i_turn_left = false;
      motor1_data.i_turn_right = false;
      motor2_data.i_turn_right = false;

      turning_left = false;
      turning_right = false;
    }
     // Wait until turn is done
  }

  for (uint8_t i = 0; i < tsp; i++) {
    if (slam_tof_data[i].distance < 220 && slam_tof_data[i].distance > 50) {
      Serial.println("obstacle detected");

      if (slam_tof_data[i].degree <= 90 && !turning_right) {
        motor1_data.i_turn_left = true;
        motor2_data.i_turn_left = true;

        Serial.println("Turn left");

        turning_left = true;
        turn_start_time = millis();
      } else if (slam_tof_data[i].degree > 90 && !turning_left) {
        motor1_data.i_turn_right = true;
        motor2_data.i_turn_right = true;

        Serial.println("Turn right");

        turning_right = true;
        turn_start_time = millis();
      }

      // Stop forward movement
      motor1_data.i_forward = false;
      motor2_data.i_forward = false;

      break;
    }
  }


  if (mqtt_data_queue != nullptr) {
    static char robot_buffer[255];
    memset(robot_buffer, 0, 255);

    char action_buffer[11];
    memset(action_buffer, 0, 11);

    if (global_object_state.action == ACTION_IDLE) memcpy(action_buffer, "idle", 4);
    else if (global_object_state.action == ACTION_SEARCHING) memcpy(action_buffer, "searching", 9);
    else if (global_object_state.action == ACTION_RETRIEVING) memcpy(action_buffer, "retrieving", 10);
    else if (global_object_state.action == ACTION_FOLLOWING) memcpy(action_buffer, "following", 9);

    snprintf(robot_buffer, 255, "{ \"name\": \"%s\", \"coords\": [%0.2f, %0.2f], \"action\": \"%s\", \"network\": { \"online\": %d }, \"sensors\": { \"tof_sensor\": %d, \"magneto\": %d, \"servo\": %d }, \"motors\": [ %d, %d ] }", DEVICE_NAME, robot_data.pos.x_coord, robot_data.pos.y_coord, action_buffer, 1, tof_data.distance, robot_data.rotation, tof_data.degree, motor1_data.i_run, motor2_data.i_run);

    xQueueSend(mqtt_data_queue, robot_buffer, 10);
  }

  if (!turning_left && !turning_right) {
    // Update slam map
    update_map(slam_tof_data, slam_pos_data, tsp, psp);
    upload_map();
  }
}