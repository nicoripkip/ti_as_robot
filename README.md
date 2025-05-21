# Antdroid

## About

This robot is a member of a bigger swarm of antdroid and is created for the course TINLAB Autonomous Systems at the Rotterdam University of Applied Sciences. 


## Schematics

The schematics can be found under a folder called designs. The Schematics consists of a Fusion360 file with the chassis and an electrical schematic for the PCB. It also contains a gerber file for the PCB.


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

3. Create a config file by changing the example config file into the config file. Modify the settings in the config file to your personal settings. This step is mandatory otherwise the project will not compile.
```bash
cp ./include/config-example.hpp > ./include/config.hpp
```

4. Check if settings in platformio.ini are correct.
```bash
/platformio.ini
```

5. Compile the project with platformio
``` Bash
pio
```

6. Flash the project onto the desired ESP32
```bash
pio
```

## Author

Nico van Ommen | 1030808