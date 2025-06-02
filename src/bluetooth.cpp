#include "bluetooth.hpp"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "config.hpp"
#include <cmath>
#include <vector>
#include <algorithm>


static BLEScan* ble_scan;


// TODO: Write bluetooth rssi filter
std::vector<int> beacon_1_readings;
std::vector<int> beacon_2_readings;
std::vector<int> beacon_3_readings;
int maddrerssi = -70;


/**
 * @brief Function to calculate the distance of the RSSI
 * 
 * @param txPower The amount of dBm at 1 meter 
 * @param rssi
 * @param n
 * 
 * @return int
 */
static double calculate_distance(int txPower, int rssi, double n)
{
    // if (n == 0) n = 1.0;

    return pow(10.0, (double(txPower - rssi) / 10 * n ));
}


/**
 * @brief Calculating the error between the distance and the rssi power
 * 
 * @param rssi
 * @param txPower
 * @param d
 * 
 * @return int
 */
static double calculate_pathloss(int rssi, int txPower, double d)
{
    return double(txPower - rssi) / (10.0 * log10(d));
}


/**
 * @brief Calculate the median value of a list of numbers
 * 
 * @param input
 * 
 * @return int
 */
static int calculate_median(std::vector<int>* input)
{
    if (input->empty()) {
        Serial.println("List is empty");
        return 0;
    }

    std::sort(input->begin(), input->end());
    size_t s = input->size();

    if (s % 2 == 1) {
        return input->at(s / 2);
    } else {
        int left = input->at((s / 2) - 1);
        int right = input->at(s / 2);
        return (left + right) / 2;
    }
}


/**
 * @brief Take an exponential filter of values
 * 
 * @param measure
 * @param prev_measure
 * @param alpha
 * 
 * @return int
 */
static int calculate_exponential_filter(int measure, int prev_measure, double alpha)
{
    return alpha * measure + (1 - alpha) * prev_measure;
}


/**
 * @brief Class that calls a callback function
 * 
 */
class BluetoothScanCallback: public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (strcmp(advertisedDevice.getName().c_str(), "")) {
            // Serial.print("Device: ");
            // Serial.print(advertisedDevice.getAddress().toString().c_str());
            // Serial.print(" | Name: ");
            // Serial.print(advertisedDevice.getName().c_str());
            // Serial.print(" | RSSI: ");
            // Serial.println(advertisedDevice.getRSSI());

            if (strcmp(advertisedDevice.getName().c_str(), "Beacon-001") == 0) {
                beacon_1_readings.push_back(advertisedDevice.getRSSI());
            } else if (strcmp(advertisedDevice.getName().c_str(), "Beacon-002") == 0) {
                beacon_2_readings.push_back(advertisedDevice.getRSSI());
            } else if (strcmp(advertisedDevice.getName().c_str(), "Beacon-003") == 0) {
                beacon_3_readings.push_back(advertisedDevice.getRSSI());
            }
        }
    }
};


/**
 * @brief
 * 
 * @param param
 */
void bluetooth_task(void *param)
{
    static int p1m = 0;
    static int p2m = 0;
    static int p3m = 0;

    static double p1l = 2.0;
    static double p2l = 2.0;
    static double p3l = 2.0;

    static double d1r = 0;
    static double d2r = 0;
    static double d3r = 0;

    double alpha = 0.5;

    BLEDevice::init(DEVICE_NAME);

    ble_scan = BLEDevice::getScan();
    ble_scan->setAdvertisedDeviceCallbacks(new BluetoothScanCallback(), true);
    ble_scan->setActiveScan(true);
    ble_scan->setInterval(100);
    ble_scan->setWindow(100);

    while (true) {
        beacon_1_readings.clear();
        beacon_2_readings.clear();
        beacon_3_readings.clear();

        Serial.println("Scanning...");
        ble_scan->start(1, false);
        ble_scan->clearResults(); // Clear RAM after scan

        int b1r = calculate_median(&beacon_1_readings);
        int b2r = calculate_median(&beacon_2_readings);
        int b3r = calculate_median(&beacon_3_readings);

        b1r = calculate_exponential_filter(b1r, p1m, alpha);
        b2r = calculate_exponential_filter(b2r, p2m, alpha);
        b3r = calculate_exponential_filter(b3r, p3m, alpha);

        p1m = b1r;
        p2m = b2r;
        p3m = b3r;

        d1r = calculate_distance(maddrerssi, b1r, p1l);
        d2r = calculate_distance(maddrerssi, b2r, p2l);
        d3r = calculate_distance(maddrerssi, b3r, p3l);

        // p1l = calculate_pathloss(b1r, maddrerssi, d1r);
        // p2l = calculate_pathloss(b2r, maddrerssi, d2r);
        // p3l = calculate_pathloss(b3r, maddrerssi, d3r);

        Serial.print("Beacon-001 with RSSI: ");
        Serial.print(b1r);
        Serial.print(" and distance: ");
        Serial.print(d1r, 3);
        Serial.println("m");
         
        Serial.print("Beacon-002 with RSSI: ");
        Serial.print(b2r);
        Serial.print(" and distance: ");
        Serial.print(d2r, 3);
        Serial.println("m");

        Serial.print("Beacon-003 with RSSI: ");
        Serial.print(b3r);
        Serial.print(" and distance: ");
        Serial.print(d3r, 3);
        Serial.println("m");
    }
}