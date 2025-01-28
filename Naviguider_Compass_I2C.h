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
#define DEBUG (0x01)						// Are we currently Debugging?

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
 * Payload Values for Meta Events (Table From Datasheet)
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


/* Naviguider Registers */
#define NAVIGUIDER_REG_CHIP_CONTROL (0x34)							// Send 0x01 to this to start the CPU
#define NAVIGUIDER_REG_MAG_RATE (0x55)								// Register that stores the Magnometer Output Rate
#define NAVIGUIDER_REG_ACCEL_RATE (0x56)							// Register that stores the Accelormeter's Output Rate
#define NAVIGUIDER_REG_GYRO_RATE (0x57)								// Register that stores the Gyro's Output Rate
#define NAVIGUIDER_REG_QRATE_DIVISOR (0x32)							// Register that stores the QRATE Divisor
#define NAVIGUIDER_REG_PRODUCT_ID (0x90)							// Stores the product ID of the Naviguider Sentral Fusion Co-Processor
#define NAVIGUIDER_REG_ALGORITHM_CONTROL (0x54)						// Register that sets our algorithmic control, see table for NAVIGUIDER_VALUE_ALGORITHM_CONTROL
#define NAVIGUIDER_REG_ENABLE_EVENTS (0x33)                         // Register that stores our Enable Events value.

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

/* Class Declaration */
class NaviguiderCompass {
	
	protected:
		Adafruit_I2CDevice *m_i2c_dev; ///< I2C bus device
		int _hostInterruptPin;         ///< Pin for the host interrupt

	public:
		// Constructor that accepts a host interrupt pin
		NaviguiderCompass(int hostInterruptPin);

		// Initialize the sensor
		bool begin(TwoWire *wire = &Wire, uint8_t address = NAVIGUIDER_ADDRESS);

		// Read accelerometer data
		bool readAccelerometer(int16_t &x, int16_t &y, int16_t &z, uint8_t &accuracy);
		
		bool enableSensor(uint8_t sensorId, uint16_t sampleRate);
		
		

		// Add additional methods HERE for other sensors as needed

	private:
		TwoWire *_wire;     							// I2C port
		uint8_t _i2cAddress; 							// I2C address of the device
		static volatile bool interruptFlag; 			// Has interrupt happened since we serviced the previous interrupt?, static so we can reference it regardless of class, volatile, because we use an ISR
		String fusionCoprocessor;						// String tells us the name of the Fusion Co-Processor Used.
		AlgorithmControlStruct algorithmControlValues;	// Saved values read from the Algorithm Control Register	
		
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
		bool configEnableEventsRegister();
		
		// Read the enable Events register
		EnableEventsStruct readEnableEventsRegister();
		
		// Print Enable Events Register
		void printEnableEventsValues();
		
		// The Interrupt handler, for when the interrupt pin get's called
		static void interruptHandler();
};


#endif    //ends the __NAVIGUIDER_COMPASS_I2C_H__ section
