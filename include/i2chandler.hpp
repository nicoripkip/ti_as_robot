#ifndef TI_AS_I2C_HANDLER_HPP
#define TI_AS_I2C_HANDLER_HPP


#include <Arduino.h>
#include <Wire.h>


void i2c_init(uint8_t channel, uint8_t scl_pin, uint8_t sda_pin);
void i2c_write_byte(TwoWire* wire, uint8_t channel, uint16_t address, uint16_t reg, byte data);
void i2c_write_buffer(TwoWire* wire, uint8_t channel, uint16_t address, uint16_t reg, byte *buffer, uint8_t len);
byte i2c_read_byte(TwoWire* wire, uint8_t channel, uint16_t address, uint16_t reg);
void i2c_read_buffer(TwoWire* wire, uint8_t channel, uint16_t address, uint16_t reg, byte* buffer, uint8_t len);

bool i2c_take_semaphore(uint8_t channel);
bool i2c_give_semaphore(uint8_t channel);


#endif