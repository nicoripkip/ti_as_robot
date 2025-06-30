#include "slam.hpp"
#include <Arduino.h>
#include <vector>
#include "BasicLinearAlgebra.h"


const uint32_t map_width = 20;
const uint32_t map_height = 20;
const uint16_t cell_size_mm = 100;


static struct SlamMapData internal_map[map_width][map_height];
static struct SlamMapData robot_data;


const uint32_t SLAM_UPDATE_RATE = 0;


// This is the state vector in which the robot will operate, this will contain x pos, y pos, theta for direction of the compass
BLA::Matrix<3, 1> x = { 0, 0, 1 };

// This is the control vector and will contain v for speed and omega for angular velocity
BLA::Matrix<1, 3> u;


/**
 * @brief Function to
 * 
 * @param distance
 * @param x
 * 
 * @return float
 */
float convert_polar_x_to_cartesian_x(float distance, float phi)
{
    return distance * cos(phi);
}


/**
 * @brief Function to calculate
 * 
 * @param distance
 * @param y
 * 
 * @return int16_t
 */
float convert_polar_y_to_cartesian_y(float distance, float phi)
{
    return distance * sin(phi);
}


/**
 * @brief Function that will init the map in which all the slam data is stored
 * 
 */
void init_map() {
    // 
    for (uint8_t i = 0; i < map_height; i++) {
        for (uint8_t j = 0; j < map_width; j++) {
            internal_map[i][j].x_coord = i;
            internal_map[i][j].y_coord = j;
            internal_map[i][j].occupied = false;
        }
    }

    // Init the base values of the robot
    robot_data.occupied = false;
    robot_data.pheromone = 0.0;
    robot_data.seen = false;
    robot_data.x_coord = 0.0;
    robot_data.y_coord = 0.0;
}


/**
 * @brief This function will update the map according to new information given
 * 
 * @param robot_coord
 * @param magneto_data
 * @param tof_data
 */
void update_map(std::vector<struct MagnetoSensorData>* slam_magneto_data, std::vector<struct TOFSensorData>* slam_tof_data, std::vector<struct robot_pos_t>* slam_pos_data)
{
    // Perform some safety checks before some pointers are empty
    if (slam_magneto_data == nullptr) return;
    if (slam_tof_data == nullptr) return;
    if (slam_pos_data == nullptr) return;

    float dx, dy;


    // Walk through the data with the least frequencies
    for (uint8_t i = 0; i < slam_magneto_data->size(); i++) {
        char buffer[300];
        memset(buffer, 0, 300);

        struct MagnetoSensorData mdata = slam_magneto_data->at(i);
        struct TOFSensorData tdata = slam_tof_data->at(i);
        struct robot_pos_t rdata = slam_pos_data->at(i);

        snprintf(buffer, 300, "%d, %d, %d, %d, %d, %d, %d, %d, %0.2f, %0.2f, %d, %d", tdata.degree, tdata.distance, tdata.scan_interval, mdata.measure_x, mdata.measure_y, mdata.measure_z, mdata.degree, mdata.scan_interval, rdata.pos.x_coord, rdata.pos.y_coord, rdata.rotation, rdata.scan_interval);
    
        Serial.println(buffer);
    }



}


/**
 * @brief Function that constantly updates the coord of the robot where it is in the physical world
 * 
 * @param steps
 * @param rotation
 * 
 * @return struct robot_pos_t
 */
struct robot_pos_t update_robot_coord(uint16_t steps, uint16_t rotation)
{
    struct robot_pos_t robot_pos;
    static struct robot_pos_t prev_pos;
    
    // Compensate for the grid rotation
    float math_angle = 90 - rotation;  
    
    // Radians makes live easier
    float angle_rad = radians(math_angle);

    // Calculate the default moving speed of the vehicle where 60 is the wheels diameter in mm
    float moving_speed = (PI * 60) / 4096;

    float distance = steps * moving_speed;

    BLA::Matrix<3, 3> m_translation = {
        1, 0, distance,
        0, 1, 0,
        0, 0, 1
    };

    BLA::Matrix<3, 3> m_rotation = {
        cos(angle_rad), -sin(angle_rad), 0,
        sin(angle_rad), cos(angle_rad), 0,
        0, 0, 1
    };

    x = m_translation * m_rotation * x;

    // Update position
    float dx = x(0) / cell_size_mm;
    float dy = x(1) / cell_size_mm;

    // Update the local position data
    robot_pos.pos.x_coord = prev_pos.pos.x_coord + dx;
    robot_pos.pos.y_coord = prev_pos.pos.y_coord + dy;
    robot_pos.rotation = rotation;

    // Push location into the robot position vector
    prev_pos = robot_pos;
    return robot_pos;
}


/**
 * @brief Function to make sure the data is good to update the
 * 
 */
void update_lidar_coord(struct lidar_data_t* data)
{

}


/**
 * @brief Function to get the robot coord data
 * 
 * @return struct SlamMapData
 */
struct SlamMapData* get_robot_coord()
{
    return &robot_data;
}