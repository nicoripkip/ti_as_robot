#ifndef TI_AS_MAGNETOSENSOR_HPP
#define TI_AS_MAGNETOSENSOR_HPP


#include <cstdint>


struct MagnetoSensorData {
    int16_t measure_x;
    int16_t measure_y;
    int16_t measure_z;
    uint16_t degree;
    uint32_t scan_interval;
};


void magneto_sensor_task(void* param);


#endif