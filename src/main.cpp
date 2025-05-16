#include <Arduino.h>
#include "config.hpp"
#include "motor.hpp"
#include "network.hpp"
#include "bluetooth.hpp"
#include <WiFi.h>


TaskHandle_t motor_task_ptr;
TaskHandle_t network_task_ptr;
TaskHandle_t bluetooth_task_ptr;



void setup()
{
  Serial.begin(115200);

  // Register all tasks needed for the bot to work
  xTaskCreatePinnedToCore(motor_task, "Motor Task", MIN_TASK_STACK_SIZE, NULL, 1, &motor_task_ptr, 1);
  xTaskCreatePinnedToCore(network_task, "Network Task", MIN_TASK_STACK_SIZE, NULL, 1, &network_task_ptr, 1);
  xTaskCreatePinnedToCore(bluetooth_task, "Bluetooth Task", MIN_TASK_STACK_SIZE, NULL, 1, &bluetooth_task_ptr, 1);
}


void loop()
{
  Serial.println("Dit print elke 3 seconde!");
  delay(3000);
}