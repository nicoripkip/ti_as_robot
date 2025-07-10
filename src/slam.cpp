#include "slam.hpp"
#include <Arduino.h>
#include "BasicLinearAlgebra.h"
#include "buffers.hpp"
#include "config.hpp"


const uint32_t map_width = 20;
const uint32_t map_height = 20;
const uint16_t cell_size_mm = 100;


static enum slam_map_data_t internal_map[map_width][map_height];


const uint32_t SLAM_UPDATE_RATE = 0;


// This is the state vector in which the robot will operate, this will contain x pos, y pos, theta for direction of the compass
BLA::Matrix<3, 1> x = { 0, 0, 1 };


struct fused_sensor_t 
{
    struct TOFSensorData                slam_tof_data[30];
    struct robot_pos_t                  slam_pos_data;
    uint16_t                            tlen;
};


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
            internal_map[i][j] = SLAM_COORD_UNKNOWN;
        }
    }
}


/**
 * @brief Combines the slam data to update the map of the robot
 * 
 * @param slam_tof_data
 * @param slam_pos_data
 */
void update_obstacle_position(struct TOFSensorData* slam_tof_data, struct robot_pos_t* slam_pos_data)
{
    if (slam_tof_data == nullptr) return;

    if (slam_pos_data == nullptr) return;

    float corrected_tof_theta, corrected_robot_theta, global_theta;
    float obstacle_x, obstacle_y;

    corrected_tof_theta = radians(slam_tof_data->degree);
    corrected_robot_theta = radians(slam_pos_data->rotation);

    global_theta = corrected_robot_theta + corrected_tof_theta;

    obstacle_x = slam_pos_data->pos.x_coord + (convert_polar_x_to_cartesian_x(slam_tof_data->distance, global_theta) / cell_size_mm);
    obstacle_y = slam_pos_data->pos.y_coord + (convert_polar_y_to_cartesian_y(slam_tof_data->distance, global_theta) / cell_size_mm);

    int coord_x = (int)floor(obstacle_x) + 10;
    int coord_y = (int)floor(obstacle_y) + 10;

    // Make sure coord x is clamped between 0 -> map width
    if (coord_x < 0) coord_x = 0;
    else if (coord_x >= map_width) coord_x = map_width - 1;

    // Make sure coord y is clamped between 0 -> map height
    if (coord_y < 0) coord_y = 0;
    else if (coord_y >= map_height) coord_y = map_height - 1;
    
    // Serial.print("x_coord: "); Serial.println(coord_x);
    // Serial.print("y_coord: "); Serial.println(coord_y);
    // Serial.print("Free heap: ");
    // Serial.println(ESP.getFreeHeap());

    internal_map[coord_y][coord_x] = SLAM_COORD_OCCUPIED;
}


/**
 * @brief Function to fuse sensor data based on the compass information
 * 
 * @param t0
 * @param t1
 * @param slam_tof_data
 * 
 * @return struct fused_sensor_t
 */
struct fused_sensor_t filter_on_position_time(uint32_t t0, uint32_t t1, struct TOFSensorData* slam_tof_data, uint16_t tlen)
{
    struct fused_sensor_t data;

    if (slam_tof_data == nullptr) {
        Serial.println("tof data empty!");    
        return data;
    }

    memset(data.slam_tof_data, 0, sizeof(data.slam_tof_data));

    uint32_t i, j;

    j = 0;
    for (i = 0; i < tlen; i++) {
        if (slam_tof_data[i].scan_interval >= t0 && slam_tof_data[i].scan_interval <= t1) {
            if (j >= 30) {
                Serial.println("Warning: slam_tof_data buffer overflow prevented");
                break;  // Stop writing to avoid overflow
            }

            data.slam_tof_data[j].degree = slam_tof_data[i].degree;
            data.slam_tof_data[j].distance = slam_tof_data[i].distance;
            data.slam_tof_data[j].scan_interval = slam_tof_data[i].scan_interval;
            j++;
        } 
    }

    // Capture the length of the array
    data.tlen = j;

    return data; 
}


/**
 * @brief Function to insert the data points into the map
 * 
 * @param fused_data
 * @param fused_len
 */
void update_map_with_fused_data(struct fused_sensor_t* fused_data, uint16_t fused_len)
{
    if (fused_data == nullptr) {
        Serial.println("The fused data array is empty");
        return;
    }

    uint16_t i, j;
    
    for (i = 0; i < fused_len; i++) {
        if (fused_data[i].tlen == 0) continue;  

        for (j = 0; j < fused_data[i].tlen; j++) {
            // Update only an obstacle when the distance is within 500 milimeters
            if (fused_data[i].slam_tof_data[j].distance <= 300) update_obstacle_position(&fused_data[i].slam_tof_data[j], &fused_data[i].slam_pos_data);
        }
    }
}
  

/**
 * @brief This function will update the map according to new information given
 * 
 * @param robot_coord
 * @param tof_data
 */
void update_map(struct TOFSensorData* slam_tof_data, struct robot_pos_t* slam_pos_data, uint16_t tlen, uint16_t plen)
{
    // Perform some safety checks before some pointers are empty
    if (slam_tof_data == nullptr) {
        Serial.println("tof data empty!");    
        return;
    }

    if (slam_pos_data == nullptr) {
        Serial.println("pos data empty!");    
        return;
    }

    static struct fused_sensor_t map_data[50];
    float dx, dy;
    uint8_t i;

    memset(map_data, 0, sizeof(map_data));

    // Walk through the data with the least frequencies
    for (i = 0; i < plen-1; i++) {

        // Make sure the buffer does not overflow
        if (i >= 50) break;

        struct fused_sensor_t sensor_point;

        sensor_point = filter_on_position_time(slam_pos_data[i].scan_interval, slam_pos_data[i+1].scan_interval, slam_tof_data, tlen);
        sensor_point.slam_pos_data.pos = slam_pos_data[i].pos;
        sensor_point.slam_pos_data.rotation = slam_pos_data[i].rotation;
        sensor_point.slam_pos_data.scan_interval = slam_pos_data[i].scan_interval;
        
        map_data[i] = sensor_point;

        // Serial.println("\n\n============================");

        // Serial.print("Henk length: ");
        // Serial.print(sensor_point.tlen);
        // Serial.print(" for step: ");
        // Serial.println(sensor_point.slam_pos_data.pos.x_coord);

        // Serial.print("sensor length: ");
        // Serial.print(map_data[i].tlen);
        // Serial.print(" for step: ");
        // Serial.println(map_data[i].slam_pos_data.pos.x_coord);
    }

    // Put those data into the map
    update_map_with_fused_data(map_data, i);
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
    robot_pos.scan_interval = micros();

    // Push location into the robot position vector
    prev_pos = robot_pos;
    return robot_pos;
}


/**
 * @brief Function to generate a string from the map data and send it to the mqtt server
 * 
 */
void upload_map()
{
    if (mqtt_map_queue != nullptr) {
        String buffer = "";

        // Build the full map string
        for (uint8_t i = 0; i < map_height; i++) {
            for (uint8_t j = 0; j < map_width; j++) {
                buffer += String(internal_map[i][j]);
            }
        }

        const size_t chunk_size = MQTT_MAX_PACk_SIZE-1;
        size_t total_length = buffer.length();
        size_t offset = 0;

        // Temporary buffer to hold each chunk (null-terminated)
        char chunk[chunk_size + 1];  // +1 for null terminator

        while (offset < total_length) {
            size_t len = (total_length - offset > chunk_size) ? chunk_size : (total_length - offset);

            // Copy substring into chunk buffer and add null terminator
            buffer.substring(offset, offset + len).toCharArray(chunk, len + 1);

            // Serial.print("Map chunck: ");
            // Serial.print(" data:  ");
            // Serial.println(chunk);

            // Send chunk to queue
            xQueueSend(mqtt_map_queue, chunk, 0);

            offset += len;
        }
    }
}