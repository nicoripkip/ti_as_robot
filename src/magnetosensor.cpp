#include "magnetosensor.hpp"
#include <Arduino.h>
#include "config.hpp"
#include "i2chandler.hpp"
#include "buffers.hpp"
#include <DFRobot_QMC5883.h>


DFRobot_QMC5883 compass(&Wire, /*I2C addr*/ MAGNETO_SENSOR_ADDRESS);


static int16_t sensor_bias_x = 0;
static int16_t sensor_bias_y = 0;
static int16_t sensor_bias_z = 0;



/**
 * @brief Function to calculate the exponential filter
 * 
 * @param alpha
 * @param measurement
 * @param prev_measurement
 * 
 * @return uint16_t
 */
uint16_t exponential_filter(float alpha, uint16_t measurement, uint16_t prev_measurment)
{
    return (uint16_t)(alpha * prev_measurment + (1 - alpha) * measurement); 
}


/**
 * @brief Function to convert the raw output of the magneto sensor
 * 
 * @param x
 * @param y
 * 
 * @return uint16_t
 */
uint16_t convert_output_to_degree(int16_t x, int16_t y)
{
    float rad = atan2((float)y, (float)x);
    float deg = rad * (180.0 / PI);

    if (deg < 0) deg += 360.0;

    return (uint16_t)deg;
}


/**
 * @brief Function to init the magneto sensor
 * 
 * 
 */
void magneto_init()
{
    // Write byte to get data from the magneto sensor

    bool err;

    err = i2c_take_semaphore(1);
    while (!err) err = i2c_take_semaphore(1);

    byte data = i2c_read_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x0D);
    if (data == 0xFF) Serial.println("Magneto sensor found!"); 

    i2c_give_semaphore(1);

    // Set sample rate to 256 (osr)
    // Set 100hz as sampling (odr)
    // Continues sampling (mode)
    // 
    byte ctrl_reg_1_value = 0b10011001;
    byte ctrl_reg_2_value = 0b00000001;

    Serial.println("Try to configure magneto sensor!");

    err = i2c_take_semaphore(1);
    while (!err) err = i2c_take_semaphore(1);

    // Write the control values for the measurements
    i2c_write_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x09, ctrl_reg_1_value);

    // Write the control values for the interrupts
    i2c_write_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x0A, ctrl_reg_2_value);

    i2c_give_semaphore(1);

    Serial.println("Magneto sensor configured!");

    // Setup buffers
    magneto_sensor_data_queue = xQueueCreate(10, sizeof(struct MagnetoSensorData));
}


/**
 * @brief Function to read data from the magneto sensor
 * 
 * @param value_x
 * @param value_y
 * @param value_z
 */
bool magneto_measurement(int16_t* value_x, int16_t* value_y, int16_t* value_z)
{
    bool err;
    byte x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;
    int16_t measure_x, measure_y, measure_z;
    uint16_t bitmask = 0xFFFF;

    err = i2c_take_semaphore(1);
    if (!err) return false;

    x_lsb = i2c_read_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x00);
    x_msb = i2c_read_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x01);
    y_lsb = i2c_read_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x02); 
    y_msb = i2c_read_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x03);
    z_lsb = i2c_read_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x04);
    z_msb = i2c_read_byte(&Wire, 1, MAGNETO_SENSOR_ADDRESS, 0x05);

    i2c_give_semaphore(1);

    measure_x = ((int16_t)x_msb<<8) | x_lsb;
    measure_y = ((int16_t)y_msb<<8) | y_lsb;
    measure_z = ((int16_t)z_msb<<8) | z_lsb;

    *value_x = measure_x;
    *value_y = measure_y;
    *value_z = measure_z;

    // Serial.print("Magneto data: [ x: ");
    // Serial.print(measure_x);
    // Serial.print(", y: ");
    // Serial.print(measure_y);
    // Serial.print(", z: ");
    // Serial.print(measure_z);
    // Serial.println("]");

    return true;
}


/**
 * @brief Function to calibrate the sensor before using it
 * 
 * @param samples
 */
void magneto_calibrate(uint16_t samples)
{
    uint16_t i = 0;
    int16_t x, y, z;
    int32_t sample_x = 0, sample_y = 0, sample_z = 0;
    struct MagnetoSensorData data[samples];

    while (i < samples) {
        if (magneto_measurement(&data[i].measure_x, &data[i].measure_y, &data[i].measure_z)) i++;
        delayMicroseconds(300);
    }

    for (i = 0; i < samples; i++) {
        sample_x += data[i].measure_x;
        sample_y += data[i].measure_y;
        sample_z += data[i].measure_z;
    }

    sensor_bias_x = 0 - (sample_x / samples);
    sensor_bias_y = 0 - (sample_y / samples);
    sensor_bias_z = 0 - (sample_z / samples);
}


/**
 * @brief Function to start the task for the magneto sensor
 * 
 * @param param
 */
void magneto_sensor_task(void* param)
{
    // magneto_init();

    // magneto_calibrate(100);

    magneto_sensor_data_queue = xQueueCreate(300, sizeof(struct MagnetoSensorData));

    bool err = false;

    while (!err) err = i2c_take_semaphore(1);

    while (!compass.begin());

    if (compass.isQMC()) {
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS);

        compass.setDataRate(QMC5883_DATARATE_100HZ);

        compass.setSamples(QMC5883_SAMPLES_8);
    }

    i2c_give_semaphore(1);

    while (true) {
        // int16_t x, y, z;
        struct MagnetoSensorData data;
        sVector_t mag;
        // static struct MagnetoSensorData prev_data = { 0, 0, 0, 0};
        // magneto_measurement(&x, &y, &z);

        // data.measure_x = x + sensor_bias_x;
        // data.measure_y = y + sensor_bias_y;
        // data.measure_z = z + sensor_bias_z;


        // Declination angle for the netherlands
        float declinationAngle = (1.0 + (0.0 / 60.0)) * (PI / 180.0);

        if (i2c_take_semaphore(1)) {
            compass.setDeclinationAngle(declinationAngle);
            mag = compass.readRaw();

            i2c_give_semaphore(1);
        }

        float rads = atan2(mag.XAxis, mag.YAxis);
        rads += declinationAngle;
        float degs = degrees(rads);
        if (degs < 0) degs += 360;

        data.measure_x = mag.XAxis;
        data.measure_y = mag.YAxis;
        data.measure_z = mag.ZAxis;

        data.degree = exponential_filter(0.5, degs, magneto_rotation);
        data.scan_interval = micros();

        magneto_rotation = data.degree;

        if (magneto_sensor_data_queue != nullptr) {
            xQueueSend(magneto_sensor_data_queue, &data, 0);
        }

        // prev_data = data;
        delay(10);
    }
}