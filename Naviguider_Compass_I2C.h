/**
 *   @file Naviguider_Compass_I2C.h
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







#ifndef __NAVIGUIDER_I2C_H__
#define __NAVIGUIDER_I2C_H__

#include <Wire.h>

/* I2C Addresses */
#define NAVIGUIDER_ADDRESS (0x28)           // The 7 Bit slave Address
#define NAVIGUIDER_ADDRESS_SHIFTED (0x50)   // The 8 Bit Shifted Address, which includes the read/write bit.

/* Virtual Sensor Register Addresses */


/** Accelerometer
 * Sample Rate:       Set By User, 0-400Hz
 * Reporting Type:    Continuous
 * Payload Values:    SInt16 X
 *                    SInt16 Y
 *                    SInt16 Z
 *                    Uint8 Accuracy
 * Description:       Device specific output data from Accelerometer sensor
 *                    Values X, Y, and Z are scaled to maximize range and resolution. To convert this
 *                    value to engineering units in meters per second squared perform the following
 *                    operation.
 *                    Value(m/s2) = x * 0.0005  Where: x is the sensor data X, Y or Z
 */
#define NAVIGUIDER_REG_ACCELEROMETER (0x01)
