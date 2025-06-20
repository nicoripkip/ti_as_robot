#include "slam.hpp"
#include <Arduino.h>


const uint32_t map_width = 2000;
const uint32_t map_height = 2000;


static struct SlamMapData internal_map[map_width][map_height];


/**
 * @brief Function that will init the map in which all the slam data is stored
 * 
 */
void init_map() {
    for (uint8_t i = 0; i < map_height; i++) {
        for (uint8_t j = 0; j < map_width; j++) {
            internal_map[i][j].x_coord = i;
            internal_map[i][j].y_coord = j;
            internal_map[i][j].occupied = false;
        }
    }
}


/**
 * @brief This function will update the map according to new information given
 * 
 */
void update_map() {

}