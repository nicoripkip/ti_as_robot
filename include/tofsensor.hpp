#ifndef TI_AS_TOFSENSOR_HPP
#define TI_AS_TOFSENSOR_HPP


#include <cstdint>


struct TOFSensorData {
    uint16_t    degree;                        // Value for which corner the servo ha turned
    uint16_t    distance;                      // The distance measured by the tof sensor
    uint32_t    scan_interval;
};


void tof_sensor_task(void* param);


#endif