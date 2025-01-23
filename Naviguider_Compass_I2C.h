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
 * Payload Size:      7 Bytes
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


/** Magnetometer
 * Sample Rate:       Set By User, 0-125Hz
 * Reporting Type:    Continuous
 * Payload Size:      7 Bytes
 * Payload Values:    SInt16 X
 *                    SInt16 Y
 *                    SInt16 Z
 *                    UInt8 Accuracy
 * Description:       Device specific output data from Magnetometer sensor
 *                    Values X, Y, and Z are scaled to maximize range and resolution. To convert this
 *                    value to engineering units in microTesla (uT) perform the following operation.
 *                    Value(uT) = x * 0.01962  Where: x is the sensor data X, Y or Z
 */
#define NAVIGUIDER_REG_MAGNETOMETER (0x02)


/** Gyroscope
 * Sample Rate:       Set By User, 0-400Hz
 * Reporting Type:    Continuous
 * Payload Size:      7 Bytes
 * Payload Values:    SInt16 X
 *                    SInt16 Y
 *                    SInt16 Z
 *                    UInt8 Accuracy
 * Description:       Device specific output data from Gyroscope sensor
 *                    Values X, Y, and Z are scaled to maximize range and resolution. To convert this
 *                    value to engineering units in radians per second perform the following operation.
 *                    Value(rps) = x * 0.0010647  Where: x is the sensor data X, Y or Z
 */
#define NAVIGUIDER_REG_GYROSCOPE (0x04)


/** Orientation
 * Sample Rate:       0-400Hz
 * Reporting Type:    Continuous
 * Payload Size:      7 Bytes
 * Payload Values:    SInt16 Yaw
 *                    SInt16 Pitch
 *                    SInt16 Roll
 *                    UInt8 Accuracy
 * Description:       Output data from Orientation sensor
 *                    Values Yaw, Pitch, and Roll are scaled to maximize range and resolution. To convert this
 *                    value to engineering units in degrees perform the following operation. Range is from 0 to 360 degrees.
 *                    Value(deg) = x * 0.010986  Where: x is the sensor data Yaw, Pitch, or Roll
 */
#define NAVIGUIDER_REG_ORIENTATION (0x03)


/** Rotation Vectors
 * Sample Rate:       0-400Hz (Geo-magnetic Rotation maximum rate is 125Hz)
 * Reporting Type:    Continuous
 * Payload size:      10 Bytes
 * Payload Values:    SInt16 QX
 *                    SInt16 QY
 *                    SInt16 QZ
 *                    SInt16 QW
 *                    SInt16 Accuracy
 * Description:       Quaternion output data from Rotation Virtual Sensors
 *                    Unit Vector Q Values are scaled to maximize range and resolution.
 *                    To calculate the unit vector: 𝑸̂ = (x / 2^14)
 *                    Where: x is the Sint16 Quaternion data QX, QY, QZ, or QW
 */
#define NAVIGUIDER_REG_ROTATION_VECTOR (0x09)             // 9-DOF (Degrees of Freedom)
#define NAVIGUIDER_REG_GAME_ROTATION (0x0F)               // 6-DOF Accel/Gyro
#define NAVIGUIDER_REG_GEO_MAG_ROTATION (0x14)            // 6-DOF Mag/Accell


/** Barometer
 * Sample Rate:       0-50Hz
 * Reporting Type:    Continuous
 * Payload size:      3 Bytes
 * Payload Values:    SInt24 Pressure
 * Description:       Output data from Barometer sensor
 *                    To convert this value to engineering units in Pascals perform the following operation.
 *                    Value(Pa) = x * 128  Where: x is the sensor data
 */
#define NAVIGUIDER_REG_BAROMETER (0x06)


/** Event Wake
 * SENSOR_TYPE ID#: 0x41 through 0x7E
 * Reporting Type:    On Change
 * Payload Size:      Same size as Virtual SensorID (SENSOR_TYPE ID# - 64)
 * Description:       A wake Event has occurred for Virtual SensorID (SENSOR_TYPE ID# - 64)
 */
#define NAVIGUIDER_REG_EVENT_WAKE_START (0x41)
#define NAVIGUIDER_REG_EVENT_WAKE_END (0x7E)


/** Meta Event Wake
 * SENSOR_TYPE ID#: 0xF8
 * Reporting Type:    On Change
 * Payload Size:      3
 * Description:       A Meta Event Wake has occurred
 */
#define NAVIGUIDER_REG_META_EVENT_WAKE (0xF8)


/** Meta Event
 * SENSOR_TYPE ID#: 0xFE
 * Reporting Type:    On Change
 * Payload Size:      3
 * Description:       A Meta Event has occurred
 */
#define NAVIGUIDER_REG_META_EVENT (0xFE)

/**
 * Payload Values for Meta Events
 * -----------------------------------------------------------------------------
 * | Value 1 – Meta Event Type ID  | Value 2                 | Value 3          |
 * -----------------------------------------------------------------------------
 * | 0x02 Sample Rate Changed      | Sensor ID               | 0                |
 * | 0x03 Power Mode Changed       | Sensor ID               | 0                |
 * | 0x04 Error                    | Error Register          | Debug State      |
 * | 0x05 Magnetic Transient       | 1 = transient detected  | 0                |
 * |                               |0 = no transient detected|                  |
 * | 0x06 Cal Status Changed       | Cal Status Value        | Trans Component  |
 * | 0x07 Stillness Changed        | 1 = now still           | 0                |
 * |                               | 0 = no longer still     |                  |
 * | 0x09 Calibration Stable       | 1 = stable              | 0                |
 * |                               | 0 = not stable          |                  |
 * | 0x0B Sensor Error             | Sensor ID               |Sensor status bits|
 * | 0x0C FIFO Overflow            | Loss count LSB          | Loss count MSB   |
 * | 0x0D Dynamic Range Changed    | Sensor ID               | 0                |
 * | 0x0E FIFO Watermark           | Bytes remaining         | 0                |
 * | 0x0F Self-Test (BIST) Results | Sensor ID               | Test results     |
 * |                               |                         | (0 = pass)       |
 * | 0x10 Initialized              | RAM version LSB         | RAM version MSB  |
 * | 0x11 Transfer Cause           | 0                       | 0                |
 * -----------------------------------------------------------------------------
 */


