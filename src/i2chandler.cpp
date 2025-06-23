#include "i2chandler.hpp"
#include <Arduino.h>
#include <Wire.h>
#include "config.hpp"


static SemaphoreHandle_t i2c_semaphore_channel_1;
static SemaphoreHandle_t i2c_semaphore_channel_2;


/**
 * @brief Function to initialize the i2c channels with there respectable semaphores
 * 
 * @param channel
 * @param scl_pin
 * @param sda_pin
 */
void i2c_init(uint8_t channel, uint8_t scl_pin, uint8_t sda_pin)
{
    if (channel == 1) {
        i2c_semaphore_channel_1 = xSemaphoreCreateBinary();
        xSemaphoreGive(i2c_semaphore_channel_1);

        Wire.begin(sda_pin, scl_pin);
        Wire.setClock(400000);
    } else if (channel == 2) {
        i2c_semaphore_channel_2 = xSemaphoreCreateBinary();
        xSemaphoreGive(i2c_semaphore_channel_2);

        Wire1.begin(sda_pin, scl_pin);
    } else {
        
    }
}


/**
 * @brief Function that will handle the taking of a semaphore for the i2c bus
 * 
 * @param channel
 */
bool i2c_take_semaphore(uint8_t channel)
{
    if ((channel == 1 && i2c_semaphore_channel_1 == nullptr) || (channel == 2 && i2c_semaphore_channel_2 == nullptr)) return false;

    BaseType_t err;

    if (channel == 1) {
        err = xSemaphoreTake(i2c_semaphore_channel_1, ( TickType_t ) 10 );
        if (err != pdTRUE) return false;
    } else if (channel == 2) {
        err = xSemaphoreTake(i2c_semaphore_channel_2, ( TickType_t ) 10 );
        if (err != pdTRUE) return false;
    } else {
        return false;
    }

    return true;
}


/**
 * @brief Function that will handle the returning of a semaphore for the i2c bus
 * 
 * @param channel
 */
bool i2c_give_semaphore(uint8_t channel) 
{
    if ((channel == 1 && i2c_semaphore_channel_1 == nullptr) || (channel == 2 && i2c_semaphore_channel_2 == nullptr)) return false;

    BaseType_t err;

    if (channel == 1) {
        err = xSemaphoreGive(i2c_semaphore_channel_1);
        if (err != pdTRUE) return false;
    } else if (channel == 2) {
        err = xSemaphoreGive(i2c_semaphore_channel_2);
        if (err != pdTRUE) return false;
    } else {
        return false;
    }

    return true;
}


/**
 * @brief Function to write a single byte over a given i2c bus
 * 
 * @param wire
 * @param address
 * @param reg
 * @param data
 */
void i2c_write_byte(TwoWire* wire, uint8_t channel, uint16_t address, uint16_t reg, byte data)
{
    if (wire == nullptr) return;

    wire->beginTransmission(address);

    // First send register address to i2c object
    wire->write(reg);

    // Then send byte to write to the register
    wire->write(data);

    wire->endTransmission(true);
}


/**
 * @brief Write a sequence of bytes of a given i2c bus
 * 
 * @param wire
 * @param address
 * @param reg
 * @param buffer
 * @param len
 */
void i2c_write_buffer(TwoWire* wire, uint8_t channel, uint16_t address, uint16_t reg, byte *buffer, uint8_t len) 
{
    if (wire == nullptr) return;

    i2c_take_semaphore(channel);

    wire->beginTransmission(address);
    
    wire->write(reg);

    wire->write(buffer, len);

    wire->endTransmission(true);

    i2c_give_semaphore(channel);
}


/**
 * @brief Function to read a single byte from a given i2c bus
 * 
 * @param wire
 * @param address
 * @param reg
 * 
 * @return byte
 */
byte i2c_read_byte(TwoWire* wire, uint8_t channel, uint16_t address, uint16_t reg)
{
    if (wire == nullptr) {
        Serial.println("err, wire is not defined!");    
        return 0;
    }

    wire->beginTransmission(address);
    
    wire->write(reg);

    wire->endTransmission(true);

    int res = wire->requestFrom(address, 1);
    if (res != 1) {
        Serial.println("err, Wire could not request data from i2c address");    
        return 0;
    }

    byte data = 0;
    if (wire->available()) {
        data = wire->read();        
    }

    return data;
}


/**
 * @brief Function to read multiple bytes from a given i2c bus
 * 
 * @param wire
 * @param channel
 * @param address
 * @param reg
 * @param buffer
 * @param len
 * 
 */
void i2c_read_buffer(TwoWire* wire, uint8_t channel, uint16_t address, uint16_t reg, byte* buffer, uint8_t len)
{
    if (wire == nullptr) {
        Serial.println("err, wire is not defined!");    
        return;
    }

    i2c_take_semaphore(channel);

    wire->beginTransmission(address);
    
    wire->write(reg);

    wire->endTransmission(true);

    int err = wire->requestFrom(address, len);
    if (err != 1) {
        Serial.println("err, Wire could not request data from i2c address");    
        return;
    }

    // Clear buffer
    memset(buffer, 0, len);

    uint8_t n = 0;
    while (wire->available()) {
         buffer[n++] = wire->read();
         
        if (n == len-1) break;
    }

    i2c_give_semaphore(channel);
}