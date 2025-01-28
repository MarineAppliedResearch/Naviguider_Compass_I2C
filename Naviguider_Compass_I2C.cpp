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
 
 // Define static member variable
volatile bool NaviguiderCompass::interruptFlag = false;

// Constructor with host interrupt pin
NaviguiderCompass::NaviguiderCompass(int hostInterruptPin)
    : _hostInterruptPin(hostInterruptPin) {
    // Set the host interrupt pin as input
    pinMode(_hostInterruptPin, INPUT);
}

// Initialize the device
bool NaviguiderCompass::begin(TwoWire *wire, uint8_t address) {
    delay(100); // Allow time for the bootloader to finish
    m_i2c_dev = new Adafruit_I2CDevice(address, wire);

    // Start I2C communication
    if (!m_i2c_dev->begin()) {
        return false; // Failed to initialize I2C device
    }
	
	bool returnVal = false;
	
	// Get Naviguider Fusion Co-processor Device ID
	fusionCoprocessor = getFusionCoprocessor();
	
	// Setup the interrupt pin, and interrupt service routine
	pinMode(2, INPUT); // Set D2 as input for interrupt
    attachInterrupt(digitalPinToInterrupt(2), NaviguiderCompass::interruptHandler, FALLING); // Attach interrupt to pin 2
	
	// Start the CPU
	startCPU();
	
	// Set the MAG, ACCEL, and GYRO Sensor Rates
	setSensorRates();
	
	// Configure the Algorithm Control Register
	configAlgorithmControlRegister();
	
	// Read the 
	algorithmControlValues = getAlgorithmControlValue();
	
	#ifdef DEBUG
	printAlgorithmControlValues();
	#endif
	
	// Enable events
	////enableEvents();
	
	
	
	// Check for errors
	#ifdef DEBUG
	////SerialUSB.print("Errors: ");
	////SerialUSB.println(checkForErrors());
	#endif
	
    return returnVal; // Initialization succeeded
}


// Called to handle interrupt on the interrupt pin.
void NaviguiderCompass::interruptHandler(){
	
	// Set the flag inside the Interrupt Service Routine
	interruptFlag = true;
	
	#ifdef DEBUG
	SerialUSB.println("Interrupt Occurred");
	#endif
}











// Read raw accelerometer data
bool NaviguiderCompass::readAccelerometer(int16_t &x, int16_t &y, int16_t &z, uint8_t &accuracy) {
    uint8_t buffer[7]; // Buffer to hold accelerometer data

    // Write the accelerometer register and then read 7 bytes of data
    uint8_t registerAddress = NAVIGUIDER_REG_ACCELEROMETER; // Register to read from
    if (!m_i2c_dev->write_then_read(&registerAddress, 1, buffer, 7)) {
        return false; // Communication error
    }

    // Parse the data from the buffer
    x = (buffer[0] << 8) | buffer[1];
    y = (buffer[2] << 8) | buffer[3];
    z = (buffer[4] << 8) | buffer[5];
    accuracy = buffer[6];

    return true; // Success
}

// Read from the FIFO
bool NaviguiderCompass::readMetaEvent(uint8_t *buffer, size_t length) {
    if (length != 3) {
        return false; // The meta event FIFO always returns 3 bytes
    }

    // Use the NAVIGUIDER_REG_META_EVENT register to access the FIFO
    uint8_t fifoRegister = NAVIGUIDER_REG_META_EVENT;

    // Write the FIFO register address and read the 3-byte payload
    if (!m_i2c_dev->write_then_read(&fifoRegister, 1, buffer, length)) {
        return false; // Communication error
    }

    return true; // Success
}

// Wait for the "Initialized" meta event
bool NaviguiderCompass::waitForInitializedMetaEvent() {
	SerialUSB.println("Waiting for initialized Meta Event");
    uint8_t buffer[3]; // Buffer for FIFO data

    // Poll the FIFO for the Initialized Meta Event
    for (int i = 0; i < 100; i++) { // Retry up to 100 times (~1 second with 10ms delay)
        if (readMetaEvent(buffer, sizeof(buffer))) {
			
			//SerialUSB.print("Meta Buffer 0:");
            //SerialUSB.println(buffer[0]); // output meta type	
			
            if (buffer[0] == NAVIGUIDER_META_EVENT_TYPE_INITIALIZED) { // Check if it's the "Initialized" Meta Event
                SerialUSB.print("Initialized Meta Event Detected! RAM Version: ");
                SerialUSB.print(buffer[1]); // RAM Version LSB
                SerialUSB.print(".");
                SerialUSB.println(buffer[2]); // RAM Version MSB
                return true; // Success
            }
        }

        delay(10); // Short delay before retrying
    }

    return false; // Timeout or failure
}

bool NaviguiderCompass::enableSensor(uint8_t sensorId, uint16_t sampleRate) {
    uint8_t config[3];
    config[0] = sensorId;                  // Sensor ID
    config[1] = (uint8_t)(sampleRate & 0xFF);  // Sample Rate LSB
    config[2] = (uint8_t)(sampleRate >> 8);   // Sample Rate MSB

    // Write to configuration register
    return m_i2c_dev->write(config, sizeof(config));
}


// Query the register for the fusion co-processor type, and interpret it to a string
String NaviguiderCompass::getFusionCoprocessor(){
	String returnVal = "NO CO-PROCESSOR FOUND";
	
	uint8_t buffer[1];
	
	uint8_t registerAddress = NAVIGUIDER_REG_PRODUCT_ID; // Register to read from
	if (!m_i2c_dev->write_then_read(&registerAddress, 1, buffer, 1)) {
        return "NO CO-PROCESSOR FOUND"; // Communication error
    }
	
	// Based on the return value, decide which co-processor was found
	switch(buffer[0])
    {
        case NAVIGUIDER_SENTRAL_PROCESSOR_PRODUCT_ID_7180:
            returnVal = "SENtral";
            break;   
        case NAVIGUIDER_SENTRAL_PROCESSOR_PRODUCT_ID_7184:
            returnVal = "SENtral-A";        
            break;   
        case NAVIGUIDER_SENTRAL_PROCESSOR_PRODUCT_ID_7186:
            returnVal = "SENtral-A2";       
            break;   
        default:
            returnVal = "NO CO-PROCESSOR FOUND";
     }  
	 
	 // Print Out Results if we are debugging
	#ifdef DEBUG
	SerialUSB.print("Fusion Co-Processor Found: ");
	SerialUSB.println(returnVal);
	#endif
	 
	 return returnVal;
}


// Start the CPU by writing 0x01 to CHIP_CONTROL_REG (0x34)
bool NaviguiderCompass::startCPU(){
	bool returnVal = false;
	
	uint8_t registerAddress = NAVIGUIDER_REG_CHIP_CONTROL;
	uint8_t value = 0x01;  
	
    if (!m_i2c_dev->write(&value, 1, true, &registerAddress, 1)) {
         // Print Out Results if we are debugging
		#ifdef DEBUG
		SerialUSB.println("Failed To Start CPU");
		#endif
		
        returnVal = false;
    }else{
		returnVal = true;
	}
	
    #ifdef DEBUG
	SerialUSB.println("CPU Started Successfully");
	#endif
	
	return returnVal;
}


// Set the sensor rates for MAG, ACCEL, and GYRO
bool NaviguiderCompass::setSensorRates(){
	bool returnVal = true;
	
	// SET MAG RATE
	uint8_t registerAddress = NAVIGUIDER_REG_MAG_RATE;
	uint8_t value = NAVIGUIDER_SENSOR_MAG_RATE; 
	
	if (!m_i2c_dev->write(&value, 1, true, &registerAddress, 1)) {
         // Print Out Results if we are debugging
		#ifdef DEBUG
		SerialUSB.print("Failed To Set MAG RATE To: ");
		SerialUSB.println(value);
		#endif
		
        returnVal = false;
    }else{
		#ifdef DEBUG
		SerialUSB.print("Successfully Set MAG RATE To: ");
		SerialUSB.println(value);
		#endif
	}
	
	
	// SET ACCEL RATE
	registerAddress = NAVIGUIDER_REG_ACCEL_RATE;
	value = NAVIGUIDER_SENSOR_ACCEL_RATE; 
	
	if (!m_i2c_dev->write(&value, 1, true, &registerAddress, 1)) {
         // Print Out Results if we are debugging
		#ifdef DEBUG
		SerialUSB.print("Failed To Set ACCEL RATE To: ");
		SerialUSB.println(value * 10);
		#endif
		
        returnVal = false;
    }else{
		#ifdef DEBUG
		SerialUSB.print("Successfully Set ACCEL RATE To: ");
		SerialUSB.println(value * 10);
		#endif
	}
	
	
	// SET GYRO RATE
	registerAddress = NAVIGUIDER_REG_GYRO_RATE;
	value = NAVIGUIDER_SENSOR_GYRO_RATE; 
	
	if (!m_i2c_dev->write(&value, 1, true, &registerAddress, 1)) {
         // Print Out Results if we are debugging
		#ifdef DEBUG
		SerialUSB.print("Failed To Set GYRO RATE To: ");
		SerialUSB.println(value * 10);
		#endif
		
        returnVal = false;
    }else{
		#ifdef DEBUG
		SerialUSB.print("Successfully Set GYRO RATE To: ");
		SerialUSB.println(value * 10);
		#endif
	}
	
	// SET QUATERNION OUTPUT RATE
	registerAddress = NAVIGUIDER_REG_QRATE_DIVISOR;
	value = NAVIGUIDER_SENSOR_QRATE_DIVISOR; 
	
	if (!m_i2c_dev->write(&value, 1, true, &registerAddress, 1)) {
         // Print Out Results if we are debugging
		#ifdef DEBUG
		SerialUSB.print("Failed To Set QRATE Divisor To: ");
		SerialUSB.println(value);
		#endif
		
        returnVal = false;
    }else{
		#ifdef DEBUG
		SerialUSB.print("Successfully Set QRATE Divisor To: ");
		SerialUSB.println(value);
		#endif
	}
	
	return returnVal;
}


// Configure the Algorithm Control Register
bool NaviguiderCompass::configAlgorithmControlRegister(){
	bool returnVal = true;
	
	// SET MAG RATE
	uint8_t registerAddress = NAVIGUIDER_REG_ALGORITHM_CONTROL;
	uint8_t value = NAVIGUIDER_VALUE_ALGORITHM_CONTROL; 
	
	if (!m_i2c_dev->write(&value, 1, true, &registerAddress, 1)) {
         // Print Out Results if we are debugging
		#ifdef DEBUG
		SerialUSB.print("Failed To Set Algorithm Control Register To: ");
		SerialUSB.println(value, HEX);
		#endif
		
        returnVal = false;
    }else{
		#ifdef DEBUG
		SerialUSB.print("Successfully Set Algorithm Control Register To [HEX]: ");
		SerialUSB.println(value, HEX);
		#endif
	}
	
	return returnVal;
}


// return, and parse the algorithm control register values
AlgorithmControlStruct NaviguiderCompass::getAlgorithmControlValue(){
	
	// Set up a struct
	AlgorithmControlStruct values = {false, false, false, false, false, false, false, false};
    uint8_t buffer[1]; // Buffer to hold the register value

    // Write the algorithm control register address and read the data
    uint8_t registerAddress = NAVIGUIDER_REG_ALGORITHM_CONTROL;
    if (!m_i2c_dev->write_then_read(&registerAddress, 1, buffer, 1)) {
        values.CommError = true; // Set communication error flag
        return values;
    }

    // Parse the bits and update the struct fields
    values.StandbyEnable = buffer[0] & 0b00000001;
    values.RawDataEnable = buffer[0] & 0b00000010;
    values.HPRoutput = buffer[0] & 0b00000100;
    values.Axis6Enable = buffer[0] & 0b00001000;
    values.ENUoutputEnable = buffer[0] & 0b00100000;
    values.DisableGyroWhenStill = buffer[0] & 0b01000000;
    values.ParameterTransfer = buffer[0] & 0b10000000;

    return values; // Return the parsed struct
}


// Print the values in the algorithm control register to one line.
void NaviguiderCompass::printAlgorithmControlValues() {
    if (algorithmControlValues.CommError) {
        SerialUSB.println("Error: Failed to read Algorithm Control register!");
        return;
    }

    SerialUSB.print("Algorithm Control Values: ");
    SerialUSB.print("StandbyEnable=");
    SerialUSB.print(algorithmControlValues.StandbyEnable ? "Enabled" : "Disabled");
    SerialUSB.print(", RawDataEnable=");
    SerialUSB.print(algorithmControlValues.RawDataEnable ? "Enabled" : "Disabled");
    SerialUSB.print(", HPRoutput=");
    SerialUSB.print(algorithmControlValues.HPRoutput ? "Enabled" : "Disabled");
    SerialUSB.print(", 6-AxisEnable=");
    SerialUSB.print(algorithmControlValues.Axis6Enable ? "Enabled" : "Disabled");
    SerialUSB.print(", ENUoutputEnable=");
    SerialUSB.print(algorithmControlValues.ENUoutputEnable ? "Enabled" : "Disabled");
    SerialUSB.print(", DisableGyroWhenStill=");
    SerialUSB.print(algorithmControlValues.DisableGyroWhenStill ? "Enabled" : "Disabled");
    SerialUSB.print(", ParameterTransfer=");
    SerialUSB.print(algorithmControlValues.ParameterTransfer ? "Enabled" : "Disabled");
    SerialUSB.println();
}


