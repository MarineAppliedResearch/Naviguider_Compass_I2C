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


#ifndef __NAVIGUIDER_COMPASS_I2C_H__
#define __NAVIGUIDER_COMPASS_I2C_H__

#include <Adafruit_I2CDevice.h>
#include <Arduino.h>
#include <Wire.h>


/* DEBUG */
//#define DEBUG (0x01)						// Are we currently Debugging?

//#define DEBUG_LEVEL_2 (0x01)

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
#define NAVIGUIDER_REG_ACCELEROMETER_WAKE (0x41)


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
#define NAVIGUIDER_REG_MAGNETOMETER_WAKE (0x42)


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
#define NAVIGUIDER_REG_GYROSCOPE_WAKE (0x44)


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
#define NAVIGUIDER_REG_ORIENTATION_WAKE (0x43)


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

#define NAVIGUIDER_SENSOR_TYPE_TIMESTAMP (0xFC) //252
#define NAVIGUIDER_SENSOR_TYPE_TIMESTAMP_WAKE (0xF6) //246
#define NAVIGUIDER_SENSOR_TYPE_TIMESTAMP_OVERFLOW (0xFD) //253
#define NAVIGUIDER_SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE  (0xF7)  //247

/**
 * Payload Values for Meta Events (Table From Datasheet)
 * -----------------------------------------------------------------------------
 * | Value 1 – Meta Event Type ID  | Value 2                 | Value 3          |
 * -----------------------------------------------------------------------------
 * | 0x01 Flush Complete           | Sensor TYpe from Fifo_Flush register | NOT USED
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
#define NAVIGUIDER_META_EVENT_TYPE_FLUSH_COMPLETE (0x01)
#define NAVIGUIDER_META_EVENT_TYPE_SAMPLE_RATE_CHANGED (0x02)
#define NAVIGUIDER_META_EVENT_TYPE_POWER_MODE_CHANGED (0x03)
#define NAVIGUIDER_META_EVENT_TYPE_ERROR (0x04)
#define NAVIGUIDER_META_EVENT_TYPE_MAGNETIC_TRANSIENT (0x05)
#define NAVIGUIDER_META_EVENT_TYPE_CAL_STATUS_CHANGED (0x06)
#define NAVIGUIDER_META_EVENT_TYPE_STILLNESS_CHANGED (0x07)
#define NAVIGUIDER_META_EVENT_TYPE_CALIBRATION_STABLE (0x09)
#define NAVIGUIDER_META_EVENT_TYPE_SENSOR_ERROR (0x0B)
#define NAVIGUIDER_META_EVENT_TYPE_FIFO_OVERFLOW (0x0C)
#define NAVIGUIDER_META_EVENT_TYPE_DYNAMIC_RAGE_CHANGED (0x0D)   // Could mean range? but the datasheet says RAGE
#define NAVIGUIDER_META_EVENT_TYPE_FIFO_WATERMARK (0x0E) 
#define NAVIGUIDER_META_EVENT_TYPE_SELF_TEST_RESULTS (0x0F)
#define NAVIGUIDER_META_EVENT_TYPE_INITIALIZED (0x10)
#define NAVIGUIDER_META_EVENT_TYPE_TRANSFER_CAUSE (0x11)


/**
 * SENtral-A2 Register Map 
 * ----------------------------------------------------
 * | Register Name          	| Address   |I2C Access|
 * |----------------------------|-----------|----------|
 * | Data Transfer Area [00:03] | 0x00-0x03 | RO 	   |
 * | Data Transfer Area [04:07] | 0x04-0x07 | RO       |
 * | Data Transfer Area [08:11] | 0x08-0x0B | RO       |
 * | Data Transfer Area [12:15] | 0x0C-0x0F | RO       |
 * | Data Transfer Area [16:17] | 0x10-0x11 | RO       |
 * | Data Transfer Area [18:19] | 0x12-0x13 | RO       |
 * | Data Transfer Area [20:21] | 0x14-0x15 | RO       |
 * | Data Transfer Area [22:23] | 0x16-0x17 | RO       |
 * | Data Transfer Area [24:25] | 0x18-0x19 | RO       |
 * | Data Transfer Area [26:27] | 0x1A-0x1B | RO       |
 * | Data Transfer Area [28:29] | 0x1C-0x1D | RO       |
 * | Data Transfer Area [30:31] | 0x1E-0x1F | RO       |
 * | Data Transfer Area [32:33] | 0x20-0x21 | RO       |
 * | Data Transfer Area [34:35] | 0x22-0x23 | RO       |
 * | Data Transfer Area [36:37] | 0x24-0x25 | RO       |
 * | Data Transfer Area [38:39] | 0x26-0x27 | RO       |
 * | Data Transfer Area [40:41] | 0x28-0x29 | RO       |
 * | Data Transfer Area [42:43] | 0x2A-0x2B | RO       |
 * | Data Transfer Area [44:45] | 0x2C-0x2D | RO       |
 * | Data Transfer Area [46:47] | 0x2E-0x2F | RO       |
 * | Data Transfer Area [48:49] | 0x30-0x31 | RO       |
 * | FIFO Flush                 | 0x32      | RW       |
 * | Reserved                   | 0x33      | RW       |
 * | Chip Control               | 0x34      | RW       |
 * | Host Status                | 0x35      | RO       |
 * | Int Status                 | 0x36      | RO       |
 * | Chip Status                | 0x37      | RO       |
 * | Bytes Remaining LSB        | 0x38      | RO       |
 * | Bytes Remaining MSB        | 0x39      | RO       |
 * | Parameter Acknowledge      | 0x3A      | RO       |
 * | Saved Parameter Byte 0     | 0x3B      | RO       |
 * | Saved Parameter Byte 1     | 0x3C      | RO       |
 * | Saved Parameter Byte 2     | 0x3D      | RO       |
 * | Saved Parameter Byte 3     | 0x3E      | RO       |
 * | Saved Parameter Byte 4     | 0x3F      | RO       |
 * | Saved Parameter Byte 5     | 0x40      | RO       |
 * | Saved Parameter Byte 6     | 0x41      | RO       |
 * | Saved Parameter Byte 7     | 0x42      | RO       |
 * | Saved Parameter Byte 8     | 0x43      | RO       |
 * | Saved Parameter Byte 9     | 0x44      | RO       |
 * | Saved Parameter Byte 10    | 0x45      | RO       |
 * | Saved Parameter Byte 11    | 0x46      | RO       |
 * | Saved Parameter Byte 12    | 0x47      | RO       |
 * | Saved Parameter Byte 13    | 0x48      | RO       |
 * | Saved Parameter Byte 14    | 0x49      | RO       |
 * | Saved Parameter Byte 15    | 0x4A      | RO       |
 * | GP20                       | 0x4B      | RO       |
 * | GP21                       | 0x4C      | RO       |
 * | GP22                       | 0x4D      | RO       |
 * | GP23                       | 0x4E      | RO       |
 * | GP24                       | 0x4F      | RO       |
 * | Error Register             | 0x50      | RO       |
 * | Interrupt State            | 0x51      | RO       |
 * | Debug Value                | 0x52      | RO       |
 * | Debug State                | 0x53      | RO       |
 * | Parameter Page Select      | 0x54      | RW       |
 * | Host Interface Control     | 0x55      | RW       |
 * | GP31                       | 0x56      | RW       |
 * | GP32                       | 0x57      | RW       |
 * | GP33                       | 0x58      | RW       |
 * | GP34                       | 0x59      | RW       |
 * | GP35                       | 0x5A      | RW       |
 * | GP36                       | 0x5B      | RW       |
 * | Load Parameter Byte 0      | 0x5C      | RW       |
 * | Load Parameter Byte 1      | 0x5D      | RW       |
 * | Load Parameter Byte 2      | 0x5E      | RW       |
 * | Load Parameter Byte 3      | 0x5F      | RW       |
 * | Load Parameter Byte 4      | 0x60      | RW       |
 * | Load Parameter Byte 5      | 0x61      | RW       |
 * | Load Parameter Byte 6      | 0x62      | RW       |
 * | Load Parameter Byte 7      | 0x63      | RW       |
 * | Parameter Request          | 0x64      | RW       |
 * | GP46                       | 0x65      | RW       |
 * | GP47                       | 0x66      | RW       |
 * | GP48                       | 0x67      | RW       |
 * | GP49                       | 0x68      | RW       |
 * | GP50                       | 0x69      | RW       |
 * | GP51                       | 0x6A      | RW       |
 * | GP52                       | 0x6B      | RW       |
 * | Host IRQ Timestamp         | 0x6C-0x6F | RO       |
 * | ROM Version                | 0x70-0x73 | RO       |
 * | No Access                  | 0x74-0x8F | --       |
 * | Product ID                 | 0x90      | RO       |
 * | Revision ID                | 0x91      | RO       |
 * |         		            | 0x92      | RO       |
 * |                            | 0x94-0x95 | RW       |
 * | Upload Data                | 0x96      | RW       |
 * | CRC Host                   | 0x97-0x9A | RO       |
 * | Reset Request              | 0x9B      | RW       |
 * | No Access                  | 0x9C-0x9D | --       |
 * | Pass-Through Ready         | 0x9E      | RO       |
 * | SCL Low Cycles             | 0x9F      | RW       |
 * | Pass-Through Config        | 0xA0      | RW       |
 * ----------------------------------------------------
 */
#define NAVIGUIDER_REG_DATA_TRANSFER_00_03     (0x00)
#define NAVIGUIDER_REG_DATA_TRANSFER_04_07     (0x04)
#define NAVIGUIDER_REG_DATA_TRANSFER_08_11     (0x08)
#define NAVIGUIDER_REG_DATA_TRANSFER_12_15     (0x0C)
#define NAVIGUIDER_REG_DATA_TRANSFER_16_17     (0x10)
#define NAVIGUIDER_REG_DATA_TRANSFER_18_19     (0x12)
#define NAVIGUIDER_REG_DATA_TRANSFER_20_21     (0x14)
#define NAVIGUIDER_REG_DATA_TRANSFER_22_23     (0x16)
#define NAVIGUIDER_REG_DATA_TRANSFER_24_25     (0x18)
#define NAVIGUIDER_REG_DATA_TRANSFER_26_27     (0x1A)
#define NAVIGUIDER_REG_DATA_TRANSFER_28_29     (0x1C)
#define NAVIGUIDER_REG_DATA_TRANSFER_30_31     (0x1E)
#define NAVIGUIDER_REG_DATA_TRANSFER_32_33     (0x20)
#define NAVIGUIDER_REG_DATA_TRANSFER_34_35     (0x22)
#define NAVIGUIDER_REG_DATA_TRANSFER_36_37     (0x24)
#define NAVIGUIDER_REG_DATA_TRANSFER_38_39     (0x26)
#define NAVIGUIDER_REG_DATA_TRANSFER_40_41     (0x28)
#define NAVIGUIDER_REG_DATA_TRANSFER_42_43     (0x2A)
#define NAVIGUIDER_REG_DATA_TRANSFER_44_45     (0x2C)
#define NAVIGUIDER_REG_DATA_TRANSFER_46_47     (0x2E)
#define NAVIGUIDER_REG_DATA_TRANSFER_48_49     (0x30)
#define NAVIGUIDER_REG_FIFO_FLUSH              (0x32)
#define NAVIGUIDER_REG_RESERVED                (0x33)
#define NAVIGUIDER_REG_CHIP_CONTROL            (0x34)			// Send 0x01 to this to start the CPU
#define NAVIGUIDER_REG_HOST_STATUS             (0x35)
#define NAVIGUIDER_REG_INT_STATUS              (0x36)
#define NAVIGUIDER_REG_CHIP_STATUS             (0x37)
#define NAVIGUIDER_REG_BYTES_REMAINING_LSB     (0x38)			// LSB of remaining bytes in FIFO
#define NAVIGUIDER_REG_BYTES_REMAINING_MSB     (0x39)			// MSB of remaining bytes in FIFO
#define NAVIGUIDER_REG_PARAMETER_ACKNOWLEDGE   (0x3A)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_0      (0x3B)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_1      (0x3C)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_2      (0x3D)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_3      (0x3E)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_4      (0x3F)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_5      (0x40)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_6      (0x41)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_7      (0x42)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_8      (0x43)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_9      (0x44)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_10     (0x45)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_11     (0x46)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_12     (0x47)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_13     (0x48)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_14     (0x49)
#define NAVIGUIDER_REG_SAVED_PARAM_BYTE_15     (0x4A)
#define NAVIGUIDER_REG_GP20                    (0x4B)
#define NAVIGUIDER_REG_GP21                    (0x4C)
#define NAVIGUIDER_REG_GP22                    (0x4D)
#define NAVIGUIDER_REG_GP23                    (0x4E)
#define NAVIGUIDER_REG_GP24                    (0x4F)
#define NAVIGUIDER_REG_ERROR                   (0x50)
#define NAVIGUIDER_REG_INTERRUPT_STATE         (0x51)
#define NAVIGUIDER_REG_DEBUG_VALUE             (0x52)
#define NAVIGUIDER_REG_DEBUG_STATE             (0x53)
#define NAVIGUIDER_REG_PARAMETER_PAGE_SELECT   (0x54)
#define NAVIGUIDER_REG_HOST_INTERFACE_CONTROL  (0x55)
#define NAVIGUIDER_REG_GP31                    (0x56)
#define NAVIGUIDER_REG_GP32                    (0x57)
#define NAVIGUIDER_REG_GP33                    (0x58)
#define NAVIGUIDER_REG_GP34                    (0x59)
#define NAVIGUIDER_REG_GP35                    (0x5A)
#define NAVIGUIDER_REG_GP36                    (0x5B)
#define NAVIGUIDER_REG_LOAD_PARAM_BYTE_0       (0x5C)
#define NAVIGUIDER_REG_LOAD_PARAM_BYTE_1       (0x5D)
#define NAVIGUIDER_REG_LOAD_PARAM_BYTE_2       (0x5E)
#define NAVIGUIDER_REG_LOAD_PARAM_BYTE_3       (0x5F)
#define NAVIGUIDER_REG_LOAD_PARAM_BYTE_4       (0x60)
#define NAVIGUIDER_REG_LOAD_PARAM_BYTE_5       (0x61)
#define NAVIGUIDER_REG_LOAD_PARAM_BYTE_6       (0x62)
#define NAVIGUIDER_REG_LOAD_PARAM_BYTE_7       (0x63)
#define NAVIGUIDER_REG_PARAMETER_REQUEST       (0x64)
#define NAVIGUIDER_REG_GP46                    (0x65)
#define NAVIGUIDER_REG_GP47                    (0x66)
#define NAVIGUIDER_REG_GP48                    (0x67)
#define NAVIGUIDER_REG_GP49                    (0x68)
#define NAVIGUIDER_REG_GP50                    (0x69)
#define NAVIGUIDER_REG_GP51                    (0x6A)
#define NAVIGUIDER_REG_GP52                    (0x6B)
#define NAVIGUIDER_REG_HOST_IRQ_TIMESTAMP      (0x6C)
#define NAVIGUIDER_REG_ROM_VERSION             (0x70)
#define NAVIGUIDER_REG_NO_ACCESS_74_8F         (0x74)
#define NAVIGUIDER_REG_PRODUCT_ID              (0x90)			// Stores the product ID of the Naviguider Sentral Fusion Co-Processor
#define NAVIGUIDER_REG_REVISION_ID             (0x91)
#define NAVIGUIDER_REG_UNKNOWN_92              (0x92)
#define NAVIGUIDER_REG_UNKNOWN_94_95           (0x94)
#define NAVIGUIDER_REG_UPLOAD_DATA             (0x96)
#define NAVIGUIDER_REG_CRC_HOST                (0x97)
#define NAVIGUIDER_REG_RESET_REQUEST           (0x9B)
#define NAVIGUIDER_REG_NO_ACCESS_9C_9D         (0x9C)
#define NAVIGUIDER_REG_PASS_THROUGH_READY      (0x9E)
#define NAVIGUIDER_REG_SCL_LOW_CYCLES          (0x9F)
#define NAVIGUIDER_REG_PASS_THROUGH_CONFIG     (0xA0)


/** NOW UNSURE ABOUT THESE VALUES						
#define NAVIGUIDER_REG_MAG_RATE (0x55)								// Register that stores the Magnometer Output Rate
#define NAVIGUIDER_REG_ACCEL_RATE (0x56)							// Register that stores the Accelormeter's Output Rate
#define NAVIGUIDER_REG_GYRO_RATE (0x57)								// Register that stores the Gyro's Output Rate
#define NAVIGUIDER_REG_QRATE_DIVISOR (0x32)							// Register that stores the QRATE Divisor
#define NAVIGUIDER_REG_ALGORITHM_CONTROL (0x54)						// Register that sets our algorithmic control, see table for NAVIGUIDER_VALUE_ALGORITHM_CONTROL
#define NAVIGUIDER_REG_ENABLE_EVENTS (0x33)                         // Register that stores our Enable Events value.
*/



/* Register Values */

// Sentral Fusion Co-Processor Product ID's
#define NAVIGUIDER_SENTRAL_PROCESSOR_PRODUCT_ID_7180 (0x80)     // SENtral Fusion Co-Processor
#define NAVIGUIDER_SENTRAL_PROCESSOR_PRODUCT_ID_7184 (0x84)		// SENtral-A Fusion Co-Processor
#define NAVIGUIDER_SENTRAL_PROCESSOR_PRODUCT_ID_7186 (0x86)		// Sentral-A2 Fusion Co-Processor
#define NAVIGUIDER_SENSOR_MAG_RATE (0x64)						// In Hertz, 0x64 = 100Hz
#define NAVIGUIDER_SENSOR_ACCEL_RATE (0x0A)						// In Hertz / 10, 0x0A = 100Hz/10 
#define NAVIGUIDER_SENSOR_GYRO_RATE (0x0F)						// In Hertz / 10, 0x0F = 150Hz/10
#define NAVIGUIDER_SENSOR_QRATE_DIVISOR (0x01)					// 

/*
 * AlgorithmControl Register (0x54)
 * NAVIGUIDER_VALUE_ALGORITHM_CONTROL
 * --------------------------------------------------------------
 * [0] StandbyEnable          : 1 = Enable Standby state
 * [1] RawDataEnable          : 1 = Raw data provided in MX, MY,
 *                                MZ, AX, AY, AZ, GX, GY, & GZ.
 *                                0 = Scaled sensor data.
 * [2] HPRoutput              : 1 = Heading, pitch, and roll output in QX,
 *                                QY, & QZ. QW = 0.0.
 *                                0 = Quaternion outputs.
 * [3] 6-AxisEnable           : 1 = 6-axis sensor fusion
 *                                0 = 9-axis sensor fusion
 *                                (rev 1.2 or higher firmware only)
 * [5] ENUoutputEnable        : 1 = ENU output
 *                                0 = NED output
 * [6] DisableGyroWhenStill   : 1 = Gyro off during stillness
 *                                0 = Gyro stays on during stillness.
 * [7] Parameter Transfer     : 1 = Initiate Parameter Transfer
 *                                0 = Terminate Parameter Transfer
 * --------------------------------------------------------------
 */
#define NAVIGUIDER_VALUE_ALGORITHM_CONTROL (0x06)			// 0x06 is RawDataEnable, and HPROutput


/*
 * EnableEvents Register (0x33)
 * NAVIGUIDER_VALUE_ENABLE_EVENTS
 * --------------------------------------------------------------
 * A value of '1' indicates an interrupt to the host will be 
 * generated for the corresponding event.
 *
 * [0] CPUReset          : Non-maskable
 * [1] Error             : Error event
 * [2] QuaternionResult  : Quaternion result available
 * [3] MagResult         : Magnetometer result available
 * [4] AccelResult       : Accelerometer result available
 * [5] GyroResult        : Gyroscope result available
 * [6] Reserved          : Reserved for future use
 * [7] Reserved          : Reserved for future use
 * --------------------------------------------------------------
 */
#define NAVIGUIDER_VALUE_ENABLE_EVENTS (0x3F)			// 0x3F is ALL on but the reserved values


/**
 * SENtral-A2 Parameter Page Select (Register 0x54)
 * --------------------------------------------------------------
 * | Bits 7-4 | Parameter Size (Transfer Size in Bytes)         |
 * | Bits 3-0 | Parameter Page (Selects Parameter Page Number)  |
 * --------------------------------------------------------------
 * 
 * Parameter Page Table:
 * --------------------------------------------------------------
 * | Page | Name              | Description                     |
 * |------|------------------|----------------------------------|
 * | 0x0  | None             | Must write this after accessing  |
 * |      |                  | Algorithm Page to signal safety. |
 * | 0x1  | System           | Controls system-wide settings,   |
 * |      |                  | meta events, FIFO watermark, etc.|
 * | 0x2  | Algorithm        | Contains sensor fusion warm      |
 * |      |                  | start parameters for saving and  |
 * |      |                  | restoring calibration data.      |
 * | 0x3  | Sensor Info      | Contains parameters for real &   |
 * |      |                  | virtual sensors, including their |
 * |      |                  | status & configuration.          |
 * | 0x4  | Reserved         | Reserved for future use.         |
 * | 0x5  | Sensor Config    | Allows writing & reading sensor  |
 * |      |                  | configurations (both wakeup &    |
 * |      |                  | non-wakeup sensors).             |
 * | 0x6  | Reserved         | Reserved for future use.         |
 * | 0x7  | Reserved         | Reserved for future use.         |
 * | 0x8  | Reserved         | Reserved for future use.         |
 * | 0x9  | Customer Page 1  | Reserved for customer use.       |
 * | 0xA  | Customer Page 2  | Reserved for customer use.       |
 * | 0xB  | Customer Page 3  | Reserved for customer use.       |
 * | 0xC  | Customer Page 4  | Reserved for customer use.       |
 * | 0xD  | Algorithm Knobs  | Allows configuration of advanced |
 * |      |                  | features of the sensor fusion.   |
 * | 0xE  | Reserved         | Reserved for future use.         |
 * | 0xF  | Reserved         | Reserved for future use.         |
 * --------------------------------------------------------------
 * 
 * Notes:
 * - The **most significant nibble (Bits 7-4)** sets the transfer size.
 *   - If set to 0, the **default** transfer size is selected:
 *     - 16 bytes for saving/reading.
 *     - 8 bytes for loading/writing.
 *   - If a size larger than the default is selected, it will be limited to 
 *     16 bytes (saving/reading) or 8 bytes (loading/writing).
 * 
 * - The **least significant nibble (Bits 3-0)** selects the parameter page.
 */
#define NAVIGUIDER_PARAMETER_PAGE_NONE            (0x00)  // Must write this after accessing Algorithm Page
#define NAVIGUIDER_PARAMETER_PAGE_SYSTEM          (0x01)  // System-wide settings, meta events, FIFO watermark, etc.
#define NAVIGUIDER_PARAMETER_PAGE_ALGORITHM       (0x02)  // Sensor fusion algorithm warm start parameters
#define NAVIGUIDER_PARAMETER_PAGE_SENSOR_INFO     (0x03)  // Sensor parameters (real or virtual) for reading status/configuration
#define NAVIGUIDER_PARAMETER_PAGE_RESERVED_4      (0x04)  // Reserved for future use
#define NAVIGUIDER_PARAMETER_PAGE_SENSOR_CONFIG   (0x05)  // Sensor configuration for writing Sensor Configuration structures
#define NAVIGUIDER_PARAMETER_PAGE_RESERVED_6      (0x06)  // Reserved for future use
#define NAVIGUIDER_PARAMETER_PAGE_RESERVED_7      (0x07)  // Reserved for future use
#define NAVIGUIDER_PARAMETER_PAGE_RESERVED_8      (0x08)  // Reserved for future use
#define NAVIGUIDER_PARAMETER_PAGE_CUSTOM_9        (0x09)  // Reserved for customer-specific usage
#define NAVIGUIDER_PARAMETER_PAGE_CUSTOM_10       (0x0A)  // Reserved for customer-specific usage
#define NAVIGUIDER_PARAMETER_PAGE_CUSTOM_11       (0x0B)  // Reserved for customer-specific usage
#define NAVIGUIDER_PARAMETER_PAGE_CUSTOM_12       (0x0C)  // Reserved for customer-specific usage
#define NAVIGUIDER_PARAMETER_PAGE_ALGORITHM_KNOBS (0x0D)  // Advanced algorithm configuration parameters
#define NAVIGUIDER_PARAMETER_PAGE_RESERVED_14     (0x0E)  // Reserved for future use
#define NAVIGUIDER_PARAMETER_PAGE_RESERVED_15     (0x0F)  // Reserved for future use


/**
 * SENtral-A2 System Parameters (Parameter Page 1) NAVIGUIDER_PARAMETER_PAGE_SYSTEM
 * -------------------------------------------------------------------
 * | Parameter Number | Parameter Name                               |
 * |-----------------|----------------------------------------------|
 * | 1              | Non-wakeup FIFO Meta Event Control           |
 * | 2              | FIFO Control                                 |
 * | 3              | Sensor Status Bank 0 (Sensors 1-16)         |
 * | 4              | Sensor Status Bank 1 (Sensors 17-32)        |
 * | 5              | Sensor Status Bank 2 (Sensors 33-48)        |
 * | 6              | Sensor Status Bank 3 (Sensors 49-64)        |
 * | 7              | Sensor Status Bank 4 (Sensors 65-80)        |
 * | 8              | Sensor Status Bank 5 (Sensors 81-96)        |
 * | 9              | Sensor Status Bank 6 (Sensors 97-112)       |
 * | 10             | Sensor Status Bank 7 (Sensors 113-128)      |
 * | 11-28          | Reserved                                    |
 * | 29             | Meta Event Control for Wakeup FIFO          |
 * | 30             | Host IRQ Timestamp                          |
 * | 31             | Physical Sensor Status                      |
 * | 32             | Physical Sensors Present                    |
 * | 33-96          | Physical Sensor Info (Orientation Matrix, etc.) |
 * -------------------------------------------------------------------
 */

#define NAVIGUIDER_SYSTEM_PARAMETER_NON_WAKEUP_META_EVENT_CONTROL  (0x01)
#define NAVIGUIDER_SYSTEM_PARAMETER_FIFO_CONTROL                   (0x02)
#define NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_0           (0x03)
#define NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_1           (0x04)
#define NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_2           (0x05)
#define NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_3           (0x06)
#define NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_4           (0x07)
#define NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_5           (0x08)
#define NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_6           (0x09)
#define NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_7           (0x0A)
#define NAVIGUIDER_SYSTEM_PARAMETER_META_EVENT_CONTROL_WAKEUP_FIFO (0x1D)
#define NAVIGUIDER_SYSTEM_PARAMETER_HOST_IRQ_TIMESTAMP             (0x1E)
#define NAVIGUIDER_SYSTEM_PARAMETER_PHYSICAL_SENSOR_STATUS         (0x1F)
#define NAVIGUIDER_SYSTEM_PARAMETER_PHYSICAL_SENSORS_PRESENT       (0x20)
#define NAVIGUIDER_SYSTEM_PARAMETER_PHYSICAL_SENSOR_INFO_START     (0x21) // Range 33-96


#define NAVIGUIDER_SENSOR_TYPE_NA									0
#define NAVIGUIDER_SENSOR_TYPE_ACCELEROMETER                       1
#define NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD                      2
#define NAVIGUIDER_SENSOR_TYPE_ORIENTATION                         3
#define NAVIGUIDER_SENSOR_TYPE_GYROSCOPE                           4
#define NAVIGUIDER_SENSOR_TYPE_LIGHT                               5
#define NAVIGUIDER_SENSOR_TYPE_PRESSURE                            6
#define NAVIGUIDER_SENSOR_TYPE_TEMPERATURE                         7
#define NAVIGUIDER_SENSOR_TYPE_PROXIMITY                           8
#define NAVIGUIDER_SENSOR_TYPE_GRAVITY                             9
#define NAVIGUIDER_SENSOR_TYPE_LINEAR_ACCELERATION                 10
#define NAVIGUIDER_SENSOR_TYPE_ROTATION_VECTOR                     11
#define NAVIGUIDER_SENSOR_TYPE_RELATIVE_HUMIDITY                   12
#define NAVIGUIDER_SENSOR_TYPE_AMBIENT_TEMPERATURE                 13
#define NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED         14
#define NAVIGUIDER_SENSOR_TYPE_GAME_ROTATION_VECTOR                15
#define NAVIGUIDER_SENSOR_TYPE_GYROSCOPE_UNCALIBRATED              16
#define NAVIGUIDER_SENSOR_TYPE_SIGNIFICANT_MOTION                  17
#define NAVIGUIDER_SENSOR_TYPE_STEP_DETECTOR                       18
#define NAVIGUIDER_SENSOR_TYPE_STEP_COUNTER                        19
#define NAVIGUIDER_SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR         20
#define NAVIGUIDER_SENSOR_TYPE_HEART_RATE                          21
#define NAVIGUIDER_SENSOR_TYPE_TILT_DETECTOR                       22
#define NAVIGUIDER_SENSOR_TYPE_WAKE_GESTURE                        23
#define NAVIGUIDER_SENSOR_TYPE_GLANCE_GESTURE                      24
#define NAVIGUIDER_SENSOR_TYPE_PICK_UP_GESTURE                     25
#define NAVIGUIDER_SENSOR_TYPE_PDR                                 26
#define NAVIGUIDER_SENSOR_TYPE_RAW_ACCEL                           28  //jm 
#define NAVIGUIDER_SENSOR_TYPE_RAW_MAG                             29  //jm
#define NAVIGUIDER_SENSOR_TYPE_RAW_GYRO                            30  //jm
#define NAVIGUIDER_SENSOR_TYPE_ACTIVITY                            31
#define NAVIGUIDER_SENSOR_TYPE_CAR_MAG_DATA                        32
#define NAVIGUIDER_SENSOR_TYPE_VISIBLE_END                         63


 // Define wake versions of sensors
#define NAVIGUIDER_SENSOR_TYPE_ACCELEROMETER_WAKE         65
#define NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD_WAKE        66
#define NAVIGUIDER_SENSOR_TYPE_ORIENTATION_WAKE           67
#define NAVIGUIDER_SENSOR_TYPE_GYROSCOPE_WAKE             68
#define NAVIGUIDER_SENSOR_TYPE_LIGHT_WAKE                 69
#define NAVIGUIDER_SENSOR_TYPE_PRESSURE_WAKE              70
#define NAVIGUIDER_SENSOR_TYPE_TEMPERATURE_WAKE           71
#define NAVIGUIDER_SENSOR_TYPE_PROXIMITY_WAKE             72
#define NAVIGUIDER_SENSOR_TYPE_GRAVITY_WAKE               73
#define NAVIGUIDER_SENSOR_TYPE_LINEAR_ACCEL_WAKE          74
#define NAVIGUIDER_SENSOR_TYPE_ROTATION_VECTOR_WAKE       75
#define NAVIGUIDER_SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE     76
#define NAVIGUIDER_SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE   77
#define NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD_UNCAL_WAKE  78
#define NAVIGUIDER_SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE  79
#define NAVIGUIDER_SENSOR_TYPE_GYROSCOPE_UNCAL_WAKE       80
#define NAVIGUIDER_SENSOR_TYPE_SIGNIFICANT_MOTION_WAKE    81
#define NAVIGUIDER_SENSOR_TYPE_STEP_DETECTOR_WAKE         82
#define NAVIGUIDER_SENSOR_TYPE_STEP_COUNTER_WAKE          83
#define NAVIGUIDER_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR_WAKE 84
#define NAVIGUIDER_SENSOR_TYPE_CAR_DETECT_WAKE            85
#define NAVIGUIDER_SENSOR_TYPE_TILT_DETECTOR_WAKE         86

#define NAVIGUIDER_PARAM_TRANSFER_BIT                  0x80  // Bit 7 in Algorithm Control Register

/* STRUCT Definitions */

/* AlgorithmControlStruct Struct */
struct AlgorithmControlStruct {
    bool StandbyEnable;
    bool RawDataEnable;
    bool HPRoutput;
    bool Axis6Enable;
    bool ENUoutputEnable;
    bool DisableGyroWhenStill;
    bool ParameterTransfer;
    bool CommError; // Set to true if communication fails
};


/* EnableEventsStruct Struct */
struct EnableEventsStruct {
    bool CPUReset;
    bool Error;
    bool QuaternionResult;
    bool MagResult;
    bool AccelResult;
    bool GyroResult;
    bool ReservedA;
    bool ReservedB;
};

// Struct to represent parsed sensor events
struct SensorEventsStruct {
    bool CPUReset;        // Bit [0]
    bool Error;           // Bit [1]
    bool QuaternionResult; // Bit [2]
    bool MagResult;       // Bit [3]
    bool AccelResult;     // Bit [4]
    bool GyroResult;      // Bit [5]
};


/**
 * @brief Represents the status of a sensor in the Naviguider system.
 *
 * This struct provides a compact representation of a sensor's status using bit fields.
 * Each field corresponds to a specific status flag or mode, allowing efficient access.
 *
 * Fields:
 * - `IsDataAvailable` (1 bit) - Indicates if new data is available (1 = yes, 0 = no).
 * - `IsI2CNack` (1 bit) - Indicates if the sensor did not acknowledge an I2C transfer (1 = NACK received, 0 = ACK received).
 * - `IsDeviceIdError` (1 bit) - Indicates if a device ID mismatch error was detected (1 = error, 0 = no error).
 * - `IsTransientError` (1 bit) - Indicates if a transient error occurred (1 = error, 0 = no error).
 * - `IsDataLost` (1 bit) - Indicates if data was lost due to a FIFO overflow (1 = data lost, 0 = no data lost).
 * - `PowerMode` (3 bits) - Represents the sensor’s current power mode:
 *     - 0: Sensor Not Present
 *     - 1: Power Down
 *     - 2: Suspend
 *     - 3: Self-Test
 *     - 4: Interrupt Motion
 *     - 5: One Shot
 *     - 6: Low Power Active
 *     - 7: Active
 */
struct SensorStatusStruct {
    uint8_t IsDataAvailable : 1;   // 1-bit flag indicating if data is available
    uint8_t IsI2CNack : 1;         // 1-bit flag for I2C NACK (no acknowledgment)
    uint8_t IsDeviceIDError : 1;   // 1-bit flag for device ID mismatch error
    uint8_t IsTransientError : 1;  // 1-bit flag for transient errors
    uint8_t IsDataLost : 1;        // 1-bit flag indicating FIFO overflow/data loss
    uint8_t PowerMode : 3;         // 3-bit value for sensor power mode (0-7)
};


/**
 * @brief Represents a parameter information structure for accessing Naviguider parameters.
 *
 * This structure is used to specify the parameter number and the data size when
 * reading or writing configuration parameters to the Naviguider system.
 */
struct ParameterInformation
{
    uint8_t ParameterNumber; /**< The unique identifier of the parameter to read or write */
    uint8_t DataSize;        /**< The size of the parameter data in bytes */
};


/**
 * @brief Struct representing the configuration and status of an individual physical sensor.
 *
 * This struct stores the sample rate, dynamic range, and status flags of a single
 * physical sensor (e.g., accelerometer, gyroscope, or magnetometer).
 */
struct PhysicalSensorConfigStruct
{
    uint16_t SampleRate;       ///< The sample rate of the sensor in Hz.
    uint16_t DynamicRange;     ///< The dynamic range of the sensor.
    SensorStatusStruct Status; ///< The status flags of the sensor.
};


/**
 * @brief Struct representing the collection of all physical sensors in the system.
 *
 * This struct holds the configuration and status of each physical sensor,
 * including the accelerometer, gyroscope, and magnetometer.
 */
struct PhysicalSensorStatusStruct
{
    PhysicalSensorConfigStruct Magnetometer;  ///< Magnetometer sensor configuration and status.
    PhysicalSensorConfigStruct Accelerometer; ///< Accelerometer sensor configuration and status.
    PhysicalSensorConfigStruct Gyroscope;     ///< Gyroscope sensor configuration and status.
};


/**
 * @brief Represents information about a physical sensor, including its type, driver details, 
 *        power consumption, dynamic range, sample rate, and orientation matrix.
 */
struct PhysicalSensorInformationStruct
{
    uint8_t SensorType;           /**< Type of sensor, as defined in the Naviguider documentation */
    uint8_t DriverID;             /**< Unique identifier for the sensor driver */
    uint8_t DriverVersion;        /**< Version number of the sensor driver */
    uint8_t CurrentConsumption;   /**< Current consumption in 0.1 mA units */
    uint16_t DynamicRange;        /**< Current dynamic range setting of the sensor */
    uint8_t Flags;                /**< Bitfield indicating IRQ enable and power mode */
    uint8_t Reserved;             /**< Reserved for future use */
    uint16_t SampleRate;          /**< Current sample rate in Hz */
    uint8_t NumAxes;              /**< Number of axes the sensor supports */
    uint8_t OrientationMatrix[5]; /**< Orientation matrix for aligning sensor data */
};



/**
 * @brief Struct representing the physical sensors present in the system.
 * Each field corresponds to a specific sensor ID as defined in the Naviguider documentation.
 */
struct SensorsPresentBitmapStruct {
    bool accelerometer;                 // Sensor ID 0x01
    bool magnetometer;                  // Sensor ID 0x02
    bool orientation;                    // Sensor ID 0x03 (deprecated in Android SDK)
    bool gyroscope;                      // Sensor ID 0x04
    bool barometer;                      // Sensor ID 0x06
    bool gravity;                        // Sensor ID 0x09
    bool linear_acceleration;            // Sensor ID 0x0A
    bool rotation_vector;                // Sensor ID 0x0B (9DOF)

    bool uncalibrated_magnetometer;      // Sensor ID 0x0E
    bool game_rotation_vector;           // Sensor ID 0x0F (6DOF - Accel + Gyro)
    bool uncalibrated_gyroscope;         // Sensor ID 0x10
    bool geomagnetic_rotation_vector;    // Sensor ID 0x14 (6DOF - Accel + Mag)
    bool tilt_detector;                  // Sensor ID 0x16
} ;



/**
 * @brief Struct containing sensor names indexed by sensor ID.
 *
 * This struct stores human-readable names for each sensor type
 * as defined in the Naviguider documentation.
 */
struct NaviguiderSensorNamesStruct {
    const char* SensorNames[128] = {
        "na",                                       // 0
        "accelerometer",                            // 1
        "magnetic field",                           // 2
        "orientation",                              // 3
        "gyroscope",                                // 4
        "light",                                    // 5
        "pressure",                                 // 6
        "temperature",                              // 7
        "proximity",                                // 8
        "gravity",                                  // 9
        "linear acceleration",                      // 10
        "rotation vector",                          // 11
        "relative humidity",                        // 12
        "ambient temperature",                      // 13
        "magnetic field uncalibrated",              // 14
        "game rotation vector",                     // 15
        "gyroscope uncalibrated",                   // 16
        "significant motion",                       // 17
        "step detector",                            // 18
        "step counter",                             // 19
        "geomagnetic rotation vector",              // 20
        "heart rate -OR- car detector",             // 21
        "tilt detector",                            // 22
        "wake gesture",                             // 23
        "glance gesture",                           // 24
        "pick up gesture",                          // 25
        "custom_26",                                // 26
        "custom_27",                                // 27
        "raw accel",                                // 28
        "raw mag",                                  // 29
        "raw gyro",                                 // 30
        "activity",                                 // 31
        "car detect mag data (uT)",                 // 32
        "custom_33",                                // 33
        "custom_34",                                // 34
        "custom_35",                                // 35
        "custom_36",                                // 36
        "custom_37",                                // 37
        "custom_38",                                // 38
        "custom_39",                                // 39
        "custom_40",                                // 40
        "custom_41",                                // 41
        "custom_42",                                // 42
        "custom_43",                                // 43
        "custom_44",                                // 44
        "custom_45",                                // 45
        "custom_46",                                // 46
        "custom_47",                                // 47
        "custom_48",                                // 48
        "custom_49",                                // 49
        "custom_50",                                // 50
        "custom_51",                                // 51
        "custom_52",                                // 52
        "custom_53",                                // 53
        "custom_54",                                // 54
        "custom_55",                                // 55
        "custom_56",                                // 56
        "custom_57",                                // 57
        "custom_58",                                // 58
        "custom_59",                                // 59
        "custom_60",                                // 60
        "custom_61",                                // 61
        "custom_62",                                // 62
        "custom_63",                                // 63
        "reserved",                                 // 64
        "accelerometer wake",                       // 65
        "magnetic field wake",                      // 66
        "orientation wake",                         // 67
        "gyroscope wake",                           // 68
        "light wake",                               // 69
        "pressure wake",                            // 70
        "temperature wake",                         // 71
        "proximity wake",                           // 72
        "gravity wake",                             // 73
        "linear acceleration wake",                 // 74
        "rotation vector wake",                     // 75
        "relative humidity wake",                   // 76
        "ambient temperature wake",                 // 77
        "magnetic field uncalibrated wake",         // 78
        "game rotation vector wake",                // 79
        "gyroscope uncalibrated wake",              // 80
        "significant motion wake",                  // 81
        "step detector wake",                       // 82
        "step counter wake",                        // 83
        "geomagnetic rotation vector wake",         // 84
        "car detect wake",                          // 85
        "tilt detector wake",                       // 86
        "wake gesture wake",                        // 87
        "glance gesture wake",                      // 88
        "pick up gesture wake",                     // 89
        "custom_26 wake",                           // 90
        "custom_27 wake",                           // 91
        "custom_28 wake",                           // 92
        "raw mag wake",                             // 93
        "custom_30 wake",                           // 94
        "activity wake",                            // 95
        "car detect mag data (uT) wake",            // 96
        "custom_33 wake",                           // 97
        "custom_34 wake",                           // 98
        "custom_35 wake",                           // 99
        "custom_36 wake",                           // 100
        "custom_37 wake",                           // 101
        "custom_38 wake",                           // 102
        "custom_39 wake",                           // 103
        "custom_40 wake",                           // 104
        "custom_41 wake",                           // 105
        "custom_42 wake",                           // 106
        "custom_43 wake",                           // 107
        "custom_44 wake",                           // 108
        "custom_45 wake",                           // 109
        "custom_46 wake",                           // 110
        "custom_47 wake",                           // 111
        "custom_48 wake",                           // 112
        "custom_49 wake",                           // 113
        "custom_50 wake",                           // 114
        "custom_51 wake",                           // 115
        "custom_52 wake",                           // 116
        "custom_53 wake",                           // 117
        "custom_54 wake",                           // 118
        "custom_55 wake",                           // 119
        "custom_56 wake",                           // 120
        "custom_57 wake",                           // 121
        "custom_58 wake",                           // 122
        "custom_59 wake",                           // 123
        "custom_60 wake",                           // 124
        "custom_61 wake",                           // 125
        "custom_62 wake",                           // 126
        "custom_63 wake"                            // 127
    };
};


struct SensorDescriptionStruct
{
    uint8_t sensorId;
    uint8_t driverId;
    uint8_t driverVersion;
    uint8_t power;
    uint16_t maxRange;
    uint16_t resolution;
    uint16_t maxRate;
    uint16_t fifoReserved;
    uint16_t fifoMax;
    uint8_t eventSize;
    uint8_t minRate;
} ;


struct SensorConfigurationStruct
{
    uint16_t sampleRate;
    uint16_t maxReportLatency;
    uint16_t changeSensitivity;
    uint16_t dynamicRange;
};


struct SensorData3Axis
{
	float x;
	float y;
	float z;
	float extraInfo;
} ;

struct SensorData3Axis_RAW
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t status;
};

struct SensorData4Axis
{
	float x;
	float y;
	float z;
	float w;
	float extraInfo;
} ;

struct RotationVectorRaw
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t w;
	int16_t accuracy;
} ;

/* Class Declaration */
class NaviguiderCompass {
	
	protected:
		Adafruit_I2CDevice *m_i2c_dev; ///< I2C bus device
		static NaviguiderCompass* instance; // Static pointer to instance
		int _hostInterruptPin;         ///< Pin for the host interrupt

	public:
		// Constructor that accepts a host interrupt pin
		NaviguiderCompass(int hostInterruptPin);

		// Initialize the sensor
		bool begin(TwoWire *wire = &Wire, uint8_t address = NAVIGUIDER_ADDRESS);
		
		// To Be Called Every Loop. Reads the sensor event's, and processes the incoming sensor data.
		void readSensors();

		float getHeading();
		float getPitch();
		float getRoll();
		float getHeadingAccuracy();
		float getYawRate();
		
		

		// Add additional methods HERE for other sensors as needed

	private:
		TwoWire *_wire;     							// I2C port
		uint8_t _i2cAddress; 							// I2C address of the device
		static volatile bool interruptFlag; 			// Has interrupt happened since we serviced the previous interrupt?, static so we can reference it regardless of class, volatile, because we use an ISR
		String fusionCoprocessor;						// String tells us the name of the Fusion Co-Processor Used.
		static PhysicalSensorStatusStruct PhysicalSensorStatus; // Global instance holding the status of all physical sensors.
		static NaviguiderSensorNamesStruct NaviguiderSensorNames; // Keep track of sensor names vs sensor id's
		bool haveSensorInformation = false;
		uint16_t magMaxRate = 0;
		uint16_t accelMaxRate = 0;
		uint16_t gyroMaxRate = 0;
		SensorDescriptionStruct sensorInformation[128];
		SensorConfigurationStruct sensorConfiguration[127];
		ParameterInformation sensorInfoParamList[128] =
		{
			{ 0, 16 },
			{ 1, 16 },
			{ 2, 16 },
			{ 3, 16 },
			{ 4, 16 },
			{ 5, 16 },
			{ 6, 16 },
			{ 7, 16 },
			{ 8, 16 },
			{ 9, 16 },
			{ 10, 16 },
			{ 11, 16 },
			{ 12, 16 },
			{ 13, 16 },
			{ 14, 16 },
			{ 15, 16 },
			{ 16, 16 },
			{ 17, 16 },
			{ 18, 16 },
			{ 19, 16 },
			{ 20, 16 },
			{ 21, 16 },
			{ 22, 16 },
			{ 23, 16 },
			{ 24, 16 },
			{ 25, 16 },
			{ 26, 16 },
			{ 27, 16 },
			{ 28, 16 },
			{ 29, 16 },
			{ 30, 16 },
			{ 31, 16 },
			{ 32, 16 },
			{ 33, 16 },
			{ 34, 16 },
			{ 35, 16 },
			{ 36, 16 },
			{ 37, 16 },
			{ 38, 16 },
			{ 39, 16 },
			{ 40, 16 },
			{ 41, 16 },
			{ 42, 16 },
			{ 43, 16 },
			{ 44, 16 },
			{ 45, 16 },
			{ 46, 16 },
			{ 47, 16 },
			{ 48, 16 },
			{ 49, 16 },
			{ 50, 16 },
			{ 51, 16 },
			{ 52, 16 },
			{ 53, 16 },
			{ 54, 16 },
			{ 55, 16 },
			{ 56, 16 },
			{ 57, 16 },
			{ 58, 16 },
			{ 59, 16 },
			{ 60, 16 },
			{ 61, 16 },
			{ 62, 16 },
			{ 63, 16 },
			{ 64, 16 },
			{ 65, 16 },
			{ 66, 16 },
			{ 67, 16 },
			{ 68, 16 },
			{ 69, 16 },
			{ 70, 16 },
			{ 71, 16 },
			{ 72, 16 },
			{ 73, 16 },
			{ 74, 16 },
			{ 75, 16 },
			{ 76, 16 },
			{ 77, 16 },
			{ 78, 16 },
			{ 79, 16 },
			{ 80, 16 },
			{ 81, 16 },
			{ 82, 16 },
			{ 83, 16 },
			{ 84, 16 },
			{ 85, 16 },
			{ 86, 16 },
			{ 87, 16 },
			{ 88, 16 },
			{ 89, 16 },
			{ 90, 16 },
			{ 91, 16 },
			{ 92, 16 },
			{ 93, 16 },
			{ 94, 16 },
			{ 95, 16 },
			{ 96, 16 },
			{ 97, 16 },
			{ 98, 16 },
			{ 99, 16 },
			{ 100, 16 },
			{ 101, 16 },
			{ 102, 16 },
			{ 103, 16 },
			{ 104, 16 },
			{ 105, 16 },
			{ 106, 16 },
			{ 107, 16 },
			{ 108, 16 },
			{ 109, 16 },
			{ 110, 16 },
			{ 111, 16 },
			{ 112, 16 },
			{ 113, 16 },
			{ 114, 16 },
			{ 115, 16 },
			{ 116, 16 },
			{ 117, 16 },
			{ 118, 16 },
			{ 119, 16 },
			{ 120, 16 },
			{ 121, 16 },
			{ 122, 16 },
			{ 123, 16 },
			{ 124, 16 },
			{ 125, 16 },
			{ 126, 16 },
			{ 127, 16 },
		};
		
		
		AlgorithmControlStruct algorithmControlValues;	// Saved values read from the Algorithm Control Register	
		EnableEventsStruct enabledEventsValues;			// Saved values read from the Enabled Events Register
		SensorData3Axis magData;
		SensorData3Axis gyroData;
		SensorData3Axis accelData;
		SensorData3Axis orientationData;

		uint8_t fifoBuffer[24*1024]; 						// Adjust size as needed (depends on your device's FIFO size)

		
		
		/**
		 * @brief Converts a given byte array into a human-readable binary string.
		 * 
		 * @param data Pointer to the byte array.
		 * @param numBytes Number of bytes in the array.
		 * @param outputStr Pointer to a character array where the binary string will be stored.
		 *                  The caller must ensure it is at least `numBytes * 8 + 1` in size.
		 * @return Pointer to the outputStr containing the binary representation.
		 */
		char* getBinaryStringFromBytes(const void* data, uint8_t numBytes, char* outputStr);
		
		/**
		 * @brief Display the status registers of the Naviguider Compass.
		 *
		 * This function reads multiple status-related registers and prints their values 
		 * in both decimal and binary formats for debugging.
		 */
		void printNaviguiderStatusRegister();
		
		/**
		 * @brief Displays the status of all available sensors in the Naviguider system.
		 *
		 * This function queries the system parameter page for sensor status banks and
		 * prints the information in a tabular format.
		 */
		void printNaviguiderSensorStatus();
		
		
		/**
		 * @brief Displays the physical sensor status, including sample rate, dynamic range, and status flags.
		 *
		 * This function retrieves and prints the status of the physical sensors (Accelerometer, Gyroscope, Magnetometer).
		 * The output is formatted in a structured table for readability.
		 */
		void printPhysicalSensorStatus();
		
		
		/**
		 * @brief Reads the physical sensor status from the Naviguider.
		 *
		 * This function queries the Naviguider to retrieve the current status, 
		 * sample rate, and dynamic range of all physical sensors (accelerometer, 
		 * gyroscope, and magnetometer) and stores the results in the 
		 * `PhysicalSensorStatusStruct` global structure.
		 */
		void readPhysicalSensorStatus();
		
		
		/**
		 * @brief Displays the status information for a given sensor.
		 *
		 * This function prints the status of an individual sensor, including its ID,
		 * data availability, communication errors, transient errors, and power mode.
		 *
		 * @param sensorId The ID of the sensor being displayed.
		 * @param status A pointer to the SensorStatusStruct containing the sensor's status.
		 */
		void printSensorStatusRow(uint8_t sensorId, const SensorStatusStruct *status);
		
		
		/**
		 * @brief Prints detailed information about each physical sensor present in the system.
		 * 
		 * This function queries the physical sensors, retrieves their properties, and prints 
		 * them in a formatted table including sensor type, driver ID, version, power consumption, 
		 * dynamic range, sample rate, and number of axes.
		 */
		void printPhysicalSensorInformation();
		
		
		/**
		 * @brief Retrieves the name of a sensor based on its sensor ID.
		 * 
		 * This function returns a human-readable string for the given sensor ID.
		 * If the ID is out of range, it returns "unknown sensor".
		 *
		 * @param sensorId The ID of the sensor.
		 * @return A pointer to a constant string representing the sensor's name.
		 */
		const char* getSensorName(uint8_t sensorId);
		
		
		/**
		 * @brief Retrieves sensor configuration for all valid sensors.
		 *
		 * This function iterates through the sensor information array and, for each valid sensor,
		 * reads its configuration parameters from the Naviguider.
		 */
		void getSensorConfiguration();
		
		
		/**
		 * @brief Retrieves information for all sensors from the Naviguider.
		 *
		 * This function reads sensor information into the `sensorInformation` structure,
		 * then extracts and stores the maximum data rates for the magnetometer, accelerometer, and gyroscope.
		 */
		void getSensorInformation();

		void printSensorConfiguration();

		void printSensorInformation();

		/**
		 * @brief Static interrupt handler for the Naviguider Compass.
		 *
		 * This function is called when an interrupt is triggered on the host interrupt pin.
		 * Since static methods cannot access instance variables directly, this function
		 * calls the instance-specific handler via the static instance pointer.
		 */
		static void interruptHandler();

		/**
		 * @brief Instance-specific interrupt handler.
		 *
		 * This function is responsible for handling the interrupt triggered by the host interrupt pin.
		 * It sets the interrupt flag and disables further interrupts until they are re-enabled
		 * to prevent multiple triggers before the event is processed.
		 */
		void handleInterrupt();


		/**
		 * @brief Reads data from the FIFO event register.
		 *
		 * This function reads the number of bytes available in the FIFO buffer,
		 * validates the size to prevent buffer overflows, and then reads the data
		 * from the FIFO into the `fifoBuffer`.
		 *
		 * @return The number of bytes successfully read from the FIFO.
		 *         Returns 0 if no data is available or if an error occurs.
		 */
		uint32_t readFifo();


		/**
		 * @brief Retrieves the name of a meta event based on its event ID.
		 *
		 * This function returns a human-readable string for the given meta event ID.
		 * If the ID is out of the defined range, it returns "Unknown Meta Event".
		 *
		 * @param eventId The ID of the meta event.
		 * @return A pointer to a constant string representing the meta event's name.
		 */
		const char* getMetaEventName(uint8_t eventId);

		/**
		 * @brief Sets the data rate for a given sensor.
		 *
		 * This function writes the desired sample rate for a specific sensor by updating
		 * the appropriate parameter in the sensor configuration page.
		 *
		 * @param sensorId The ID of the sensor to configure.
		 * @param rate The desired sample rate in Hz.
		 * @return The status of the write operation (success or failure).
		 */
		uint32_t setSensorRate(uint8_t sensorId, uint8_t rate);


		/**
		 * @brief Converts raw 3-axis sensor data from a buffer into scaled floating-point values.
		 *
		 * This function extracts raw sensor data from the provided buffer, applies a scaling factor,
		 * and stores the converted values into a `SensorData3Axis` structure.
		 *
		 * @param[out] data Pointer to the structure where the processed sensor data will be stored.
		 * @param[in] scale The scaling factor to apply to the raw data.
		 * @param[in] buffer Pointer to the buffer containing raw sensor data (must be at least the size of `SensorData3AxisRaw`).
		 * @return Returns 1 to indicate success.
		 */
		uint8_t get3AxisSensorData(SensorData3Axis* data, float scale, uint8_t* buffer);


		/**
		 * @brief Extracts and scales a rotation vector (quaternion) from raw sensor data.
		 *
		 * This function reads raw rotation vector data from the provided buffer, applies the
		 * necessary scaling factor to convert it into a usable quaternion format, and stores
		 * the processed values in the given SensorData4Axis structure.
		 *
		 * @param rv Pointer to the SensorData4Axis structure to store the processed quaternion values.
		 * @param quaternionScale Scaling factor to convert raw quaternion values to meaningful data.
		 * @param buffer Pointer to the buffer containing raw sensor data.
		 * @return uint8_t Returns 1 to indicate success.
		 */
		uint8_t getRotationVector(SensorData4Axis* rv, float quaternionScale, uint8_t* buffer);
		
		
		// Return the fusion coprocessor string, from the registers, parsed into a string.
		String getFusionCoprocessor();
		
		// Read from the FIFO
		bool readMetaEvent(uint8_t *buffer, size_t length);

		// Wait for the "Initialized" meta event
		bool waitForInitializedMetaEvent();
		
		// Starts up the CPU
		bool startCPU();
		
		// Set the sensor Rates for MAG, ACCEL, and GYRO
		bool setSensorRates();
		
		// Configure the algorithm Control register;
		bool configAlgorithmControlRegister();
		
		// Return, and parse the algorithm control register values
		AlgorithmControlStruct getAlgorithmControlValue();
		
		// Print the algorithm control struct to one line of serial
		void printAlgorithmControlValues();
		
		// Configure the Enable Events Register
		bool configureEnabledEventsRegister();
		
		// Read the enable Events register
		EnableEventsStruct readEnabledEventsRegister();
		
		// Print Enable Events Register
		void printEnabledEventsValues();
		
		// Keep track of the timestamp the last event happened at
		uint32_t	timestamp;
		uint16_t	timestampTemp[2];
		
		
		
		// Parse the Fifo, to see what event it is
		////SensorEventsStruct parseFifo(uint32_t bytesRead);
		uint32_t parseFifo(uint32_t bytesRead);
		uint32_t parseNextFifoBlock(uint8_t* buffer, uint32_t bytesRemaining);
		
		bool enableRawSensors(bool enableMag, bool enableAccel, bool enableGyro);
		
		//bool writeParameter(uint8_t page, uint8_t paramNumber, uint8_t* data, uint8_t dataSize);
		uint32_t writeParameter(uint8_t page, const ParameterInformation* paramList, uint8_t numParams, uint8_t* values);
		bool readParameter(uint8_t page, const ParameterInformation* paramList, uint8_t numParams, uint8_t* values);
		SensorsPresentBitmapStruct querySensorsPresent();
};


#endif    //ends the __NAVIGUIDER_COMPASS_I2C_H__ section
