#include "bluetooth.hpp"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "config.hpp"
#include <cmath>


static BLEScan* ble_scan;


// TODO: Write bluetooth rssi filter


/**
 * @brief Function to calculate the distance of the RSSI
 * 
 * @param txPower The amount of dBm at 1 meter 
 * @param rssi
 * @param n
 */
static int calculate_distance(int txPower, int rssi, int n)
{
    return pow(10, ((txPower - rssi) / 10 * n ));
}


class BluetoothScanCallback: public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (strcmp(advertisedDevice.getName().c_str(), "")) {
            Serial.print("Device: ");
            Serial.print(advertisedDevice.getAddress().toString().c_str());
            Serial.print(" | Name: ");
            Serial.print(advertisedDevice.getName().c_str());
            Serial.print(" | RSSI: ");
            Serial.println(advertisedDevice.getRSSI());
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
    BLEDevice::init(DEVICE_NAME);

    ble_scan = BLEDevice::getScan();
    ble_scan->setAdvertisedDeviceCallbacks(new BluetoothScanCallback(), true);
    ble_scan->setActiveScan(true);
    ble_scan->setInterval(100);
    ble_scan->setWindow(99);

    while (true) {
        Serial.println("Scanning...");
        ble_scan->start(1, false);
        ble_scan->clearResults(); // Clear RAM after scan
        delay(5000);
    }
}