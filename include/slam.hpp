#ifndef TI_AS_SLAM_HPP
#define TI_AS_SLAM_HPP


#include <cstdint>
#include "tofsensor.hpp"
#include "magnetosensor.hpp"


struct SensorData {

};


struct SlamMapData {
    float x_coord;
    float y_coord;
    bool occupied;
    bool seen;
    float pheromone;
};


void init_map();
void update_map(struct SlamMapData* robot_coord, struct MagnetoSensorData* magneto_data, struct TOFSensorData* tof_data);


void update_robot_coord(uint16_t steps, uint16_t rotation);
struct SlamMapData get_robot_coord();


int16_t convert_polar_x_to_cartesian_x(uint16_t distance, int16_t phi);
int16_t convert_polar_y_to_cartesian_y(uint16_t distance, int16_t phi);


#endif