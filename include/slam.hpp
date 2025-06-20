#ifndef TI_AS_SLAM_HPP
#define TI_AS_SLAM_HPP


#include <cstdint>


struct SensorData {

};


struct SlamMapData {
    uint16_t x_coord;
    uint16_t y_coord;
    bool occupied;
};


void init_map();
void update_map();


#endif