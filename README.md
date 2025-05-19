# Antdroid

## About

This robot is a member of a bigger swarm of antdroid and is created for the course TINLAB Autonomous Systems at the Rotterdam University of Applied Sciences. 

## Dependencies

- <a href="https://registry.platformio.org/libraries/knolleary/PubSubClient">PubSubClient</a>
- <a href="https://registry.platformio.org/libraries/adafruit/Adafruit%20TCS34725">Adafruit TCS34725</a>

## Hardware
Here is a list of the hardware used in the robot. 

- 1x ESP32-S3
- 1x TCS34725 Color sensor
- 1x VL53L0 TOF sensor
- 1x SG90 Servo motor
- 2x 28BYJ-48 Stepper motor
- 2x Polulu A4988 Stepper motor driver
- 1x GY-271 magnetometer

## Installation
Before you can flash the code onto an ESP32 microcontroller, make sure to have <a href="#">Platformio</a> installed.

1. First clone the repository from github.
```bash
git clone 
```

2. Move into the correct folder.
```bash
cd ti_as_robot
```

3. Modify the config file under the following folder and make to fill in the correct configuration options.
```txt
/include/config.hpp
```

4. 

## Author

Nico van Ommen | 1030808