#ifndef TI_AS_COLORSENSOR_HPP
#define TI_AS_COLORSENSOR_HPP


#include <Arduino.h>


typedef struct {
    uint16_t lux;
    uint16_t temp;
    uint16_t rgb[3];
} color_data_t;


void color_sensor_task(void* param);


#endif