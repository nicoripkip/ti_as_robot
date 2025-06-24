#include "slam.hpp"
#include <Arduino.h>


const uint32_t map_width = 30;
const uint32_t map_height = 20;
const uint16_t cell_size_mm = 50;


static struct SlamMapData internal_map[map_width][map_height];
static struct SlamMapData robot_data;


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
void update_map(struct SlamMapData* robot_coord, struct MagnetoSensorData* magneto_data, struct TOFSensorData* tof_data) 
{
    // Perform some safety checks before some pointers are empty
    if (robot_coord == nullptr) return;
    if (magneto_data == nullptr) return;
    if (tof_data == nullptr) return;

    float dx, dy;
}


/**
 * @brief Function that constantly updates the coord of the robot where it is in the physical world
 * 
 * @param steps
 * @param rotation
 */
void update_robot_coord(uint16_t steps, uint16_t rotation)
{
    // Compensate for the grid rotation
    float math_angle = 90 - rotation;  
    
    // Radians makes live easier
    float angle_rad = radians(math_angle);

    // Calculate the default moving speed of the vehicle where 60 is the wheels diameter in mm
    float moving_speed = (PI * 60) / 4096;

    // Update position
    float dx = convert_polar_x_to_cartesian_x(steps * moving_speed, angle_rad) / cell_size_mm;
    float dy = -convert_polar_y_to_cartesian_y(steps * moving_speed, angle_rad) / cell_size_mm;

    robot_data.x_coord += dx;
    robot_data.y_coord += dy;
}


/**
 * @brief Function to get the robot coord data
 * 
 * @return struct SlamMapData
 */
struct SlamMapData get_robot_coord()
{
    return robot_data;
}


