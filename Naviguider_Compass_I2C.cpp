/**
 *   @file Naviguider_Compass_I2C.c
 *
 *  Library for interfacing with the Naviguider I2C Compass module.
 *
 *  Marine Applied Research & Exploration (MARE) develops and shares this 
 *  code to support the exploration and documentation of deep-water 
 *  ecosystems, contributing to their conservation and management. To 
 *  sustain our mission and initiatives, please consider donating at 
 *  https://maregroup.org/donate.
 *
 *  Created by Isaac Assegai for Marine Applied Research & Exploration.
 *
 *  This library is distributed under the BSD license. Redistribution must
 *  retain this notice and accompanying license text.
 */

 #include "Naviguider_Compass_I2C.h"

 // Constructor
NaviguiderCompass::NaviguiderCompass() {}

// Initialize the device
bool NaviguiderCompass::begin(TwoWire &wirePort, uint8_t address) {
    _wire = &wirePort;
    _i2cAddress = address;

    _wire->begin();

    // Optionally check if the device responds
    _wire->beginTransmission(_i2cAddress);
    return (_wire->endTransmission() == 0);
}

// Read raw accelerometer data
bool NaviguiderCompass::readAccelerometer(int16_t &x, int16_t &y, int16_t &z, uint8_t &accuracy) {
    _wire->beginTransmission(_i2cAddress);
    _wire->write(NAVIGUIDER_REG_ACCELEROMETER);
    if (_wire->endTransmission() != 0) {
        return false; // Error during transmission
    }

    _wire->requestFrom(_i2cAddress, (uint8_t)7); // 7 bytes for accelerometer
    if (_wire->available() < 7) {
        return false; // Not enough data
    }

    x = (_wire->read() << 8) | _wire->read();
    y = (_wire->read() << 8) | _wire->read();
    z = (_wire->read() << 8) | _wire->read();
    accuracy = _wire->read();

    return true;
}