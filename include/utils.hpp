#pragma once


#ifndef TI_AS_ARM_HPP
#define TI_AS_ARM_HPP


#include <Arduino.h>
#include <cstdint>


enum RobotAction
{
  ACTION_IDLE = 0,
  ACTION_SEARCHING = 1,
  ACTION_FOLLOWING = 2,
  ACTION_RETRIEVING = 3
};


struct object_state_t
{
    bool                found_object;
    enum RobotAction    action;
    SemaphoreHandle_t   semaphore;
};


uint32_t calculate_duty_cycle(uint16_t angle);


#endif