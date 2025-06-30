#ifndef TI_AS_SLAM_HPP
#define TI_AS_SLAM_HPP


#include <cstdint>
#include "tofsensor.hpp"
#include "magnetosensor.hpp"
#include <vector>
#include "slam.hpp"


struct map_coord_t
{
    float x_coord;
    float y_coord;
};


struct robot_pos_t 
{
    struct map_coord_t pos;
    uint16_t rotation;
    uint32_t scan_interval;
};


struct map_pos_t
{
    struct map_coord_t pos;
    bool occupied;
    bool seen;
    float pheromone;
};


struct SlamMapData {
    float x_coord;
    float y_coord;
    bool occupied;
    bool seen;
    float pheromone;
};


void init_map();
void update_map(std::vector<struct MagnetoSensorData>* slam_magneto_data, std::vector<struct TOFSensorData>* slam_tof_data, std::vector<struct robot_pos_t>* slam_pos_data);


struct robot_pos_t update_robot_coord(uint16_t steps, uint16_t rotation);
void update_lidar_coord(struct lidar_data_t* data);
struct SlamMapData* get_robot_coord();


float convert_polar_x_to_cartesian_x(float distance, float phi);
float convert_polar_y_to_cartesian_y(float distance, float phi);


#endif