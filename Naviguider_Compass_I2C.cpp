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
	
	bool returnVal = true;
	
	// Get Naviguider Fusion Co-processor Device ID
	fusionCoprocessor = getFusionCoprocessor();
	
	// Setup the interrupt pin, and interrupt service routine
	pinMode(2, INPUT); // Set D2 as input for interrupt
    attachInterrupt(digitalPinToInterrupt(2), NaviguiderCompass::interruptHandler, FALLING); // Attach interrupt to pin 2
	
	// Set the MAG, ACCEL, and GYRO Sensor Rates
	setSensorRates();
	
	// Start the CPU
	startCPU();
	
	
	
	// Configure the Algorithm Control Register
	configAlgorithmControlRegister();
	
	// Read the 
	algorithmControlValues = getAlgorithmControlValue();
	
	#ifdef DEBUG
	printAlgorithmControlValues();
	#endif
	
	// Enable events
	// Example: Enable quaternion, error, and CPU reset events
    configureEnabledEventsRegister();
	
	// Read the enabled events register
	enabledEventsValues = readEnabledEventsRegister();
	
	#ifdef DEBUG
	printEnabledEventsValues();
	#endif
	
	
	
	// Check for errors
	#ifdef DEBUG
	////SerialUSB.print("Errors: ");
	////SerialUSB.println(checkForErrors());
	#endif
	
    return returnVal; // Initialization succeeded
}


void NaviguiderCompass::readSensors() {
	int16_t ax, ay, az;
	uint8_t aaccuracy;
	readAccelerometer(ax, ay, az, aaccuracy);
	SerialUSB.print("    readAccelerometer x: "); SerialUSB.println(ax);
	SerialUSB.print("    readAccelerometer y: "); SerialUSB.println(ay);
	SerialUSB.print("    readAccelerometer z: "); SerialUSB.println(az);
	SerialUSB.print("    readAccelerometer a: "); SerialUSB.println(aaccuracy);
	
	int16_t mx, my, mz;
	uint8_t maccuracy;
	readMagnometer(mx, my, mz, maccuracy);
	SerialUSB.print("    readMagnometer x: "); SerialUSB.println(mx);
	SerialUSB.print("    readMagnometer y: "); SerialUSB.println(my);
	SerialUSB.print("    readMagnometer z: "); SerialUSB.println(mz);
	SerialUSB.print("    readMagnometer a: "); SerialUSB.println(maccuracy);
	
    uint32_t bytesRead;

    // Read the FIFO buffer
    bytesRead = readFifo();

    if (bytesRead > 0) {
        // Successfully read data from FIFO
        SerialUSB.print("Read ");
        SerialUSB.print(bytesRead);
        SerialUSB.println(" bytes from the FIFO:");

        // Parse the FIFO data into events
        SensorEventsStruct events = parseFifo(bytesRead);

        // Handle each event as needed
        if (events.CPUReset) {
            SerialUSB.println("CPU Reset Event Detected!");
            // Handle CPU reset logic
        }
        if (events.Error) {
            SerialUSB.println("Error Event Detected!");
            // Handle error logic
        }
        if (events.QuaternionResult) {
            SerialUSB.println("Quaternion Result Event Detected!");
            // Handle quaternion result logic
        }
        if (events.MagResult) {
            SerialUSB.println("Magnetometer Result Event Detected!");
            // Handle magnetometer result logic
        }
        if (events.AccelResult) {
            SerialUSB.println("Accelerometer Result Event Detected!");
            // Handle accelerometer result logic
        }
        if (events.GyroResult) {
            SerialUSB.println("Gyroscope Result Event Detected!");
            // Handle gyroscope result logic
        }

    } else {
        // No data or read error
        SerialUSB.println("Failed to read from FIFO or no data available.");
    }
	
	

    // Set the interrupt flag to false now that we've processed the event and sensor data
    interruptFlag = false;
}


// Read the fifo event register
uint32_t NaviguiderCompass::readFifo() {
    uint16_t bytesAvailable = 0;

    // Read the number of bytes available in the FIFO
    uint8_t reg = NAVIGUIDER_REG_FIFO_BYTES_REMAINING_LSB; // FIFO bytes remaining register
    if (!m_i2c_dev->write_then_read(&reg, 1, (uint8_t *)&bytesAvailable, 2)) {
        return 0; // Communication error
    }

    // Ensure bytesAvailable is within a reasonable range
    if (bytesAvailable == 0 || bytesAvailable > sizeof(fifoBuffer)) {
        return 0; // No data or invalid size
    }

    // Now read the FIFO data from the appropriate register
    uint8_t fifoDataReg = NAVIGUIDER_REG_EVENT_STATUS; // Replace with the actual FIFO data register
    if (!m_i2c_dev->write_then_read(&fifoDataReg, 1, fifoBuffer, bytesAvailable)) {
        return 0; // Read error
    }

    return bytesAvailable; // Return the total bytes read
}

SensorEventsStruct NaviguiderCompass::parseFifo(uint32_t bytesRead) {
    SensorEventsStruct events = {false, false, false, false, false, false}; // Default all events to false

    // Ensure there's data to parse
    if (bytesRead == 0 || fifoBuffer == nullptr) {
		#ifdef DEBUG
        SerialUSB.println("No data to parse in FIFO.");
		#endif
        return events;
    }

    // Process the EventStatus register (first byte of the FIFO buffer)
    uint8_t eventStatus = fifoBuffer[0]; // Assuming the first byte represents the EventStatus register

    // Parse each bit in the EventStatus register
    events.CPUReset = eventStatus & 0b00000001;          // Bit [0]
    events.Error = eventStatus & 0b00000010;             // Bit [1]
    events.QuaternionResult = eventStatus & 0b00000100; // Bit [2]
    events.MagResult = eventStatus & 0b00001000;         // Bit [3]
    events.AccelResult = eventStatus & 0b00010000;       // Bit [4]
    events.GyroResult = eventStatus & 0b00100000;        // Bit [5]

    // Debug output for parsed events
    #ifdef DEBUG
    SerialUSB.println("Parsed FIFO EventStatus:");
    SerialUSB.print("CPUReset: "); SerialUSB.println(events.CPUReset);
    SerialUSB.print("Error: "); SerialUSB.println(events.Error);
    SerialUSB.print("QuaternionResult: "); SerialUSB.println(events.QuaternionResult);
    SerialUSB.print("MagResult: "); SerialUSB.println(events.MagResult);
    SerialUSB.print("AccelResult: "); SerialUSB.println(events.AccelResult);
    SerialUSB.print("GyroResult: "); SerialUSB.println(events.GyroResult);
    #endif

    return events; // Return the parsed events
}


// Called to handle interrupt on the interrupt pin.
void NaviguiderCompass::interruptHandler(){
	
	// Set the flag inside the Interrupt Service Routine
	interruptFlag = true;
	
	#ifdef DEBUG
	SerialUSB.println("Interrupt Occurred");
	#endif
}


bool NaviguiderCompass::configureEnabledEventsRegister() {
    uint8_t registerAddress = NAVIGUIDER_REG_ENABLE_EVENTS; // EnableEvents register address
    uint8_t buffer[1] = { NAVIGUIDER_VALUE_ENABLE_EVENTS }; // Event configuration to write

    // Write the event configuration to the EnableEvents register
    if (!m_i2c_dev->write(buffer, 1, true, &registerAddress, 1)) {
        #ifdef DEBUG
        SerialUSB.println("Failed to write to EnableEvents register!");
        #endif
        return false; // Communication error
    }

    #ifdef DEBUG
    SerialUSB.print("EnableEvents register configured: 0x");
    SerialUSB.println(buffer[0], HEX); // Print the value written to the register
    #endif

    return true; // Successfully configured
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
	
	x *= 0.0005;
	y *= 0.0005;
	z *= 0.0005;

    return true; // Success
}

// Read raw accelerometer data
bool NaviguiderCompass::readMagnometer(int16_t &x, int16_t &y, int16_t &z, uint8_t &accuracy) {
    uint8_t buffer[7]; // Buffer to hold accelerometer data

    // Write the accelerometer register and then read 7 bytes of data
    uint8_t registerAddress = NAVIGUIDER_REG_MAGNETOMETER; // Register to read from
    if (!m_i2c_dev->write_then_read(&registerAddress, 1, buffer, 7)) {
        return false; // Communication error
    }

    // Parse the data from the buffer
    x = (buffer[0] << 8) | buffer[1];
    y = (buffer[2] << 8) | buffer[3];
    z = (buffer[4] << 8) | buffer[5];
    accuracy = buffer[6];
	
	x *= 0.01962;
	y *= 0.01962;
	z *= 0.01962;

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


EnableEventsStruct NaviguiderCompass::readEnabledEventsRegister() {
    EnableEventsStruct events = {false, false, false, false, false, false, false, false}; // Default all bits to false
    uint8_t registerAddress = 0x33; // Address of the EnableEvents register
    uint8_t buffer[1]; // Buffer to store the read data

    // Read the EnableEvents register
    if (!m_i2c_dev->write_then_read(&registerAddress, 1, buffer, 1)) {
        SerialUSB.println("Failed to read EnableEvents register!");
        return events; // Return the default struct with all values set to false
    }

    // Parse the bits in the buffer
    events.CPUReset = buffer[0] & 0b00000001;          // Bit [0]
    events.Error = buffer[0] & 0b00000010;             // Bit [1]
    events.QuaternionResult = buffer[0] & 0b00000100; // Bit [2]
    events.MagResult = buffer[0] & 0b00001000;         // Bit [3]
    events.AccelResult = buffer[0] & 0b00010000;       // Bit [4]
    events.GyroResult = buffer[0] & 0b00100000;        // Bit [5]
    events.ReservedA = buffer[0] & 0b01000000;         // Bit [6]
    events.ReservedB = buffer[0] & 0b10000000;         // Bit [7]

    return events; // Return the populated struct
}

void NaviguiderCompass::printEnabledEventsValues() {

    SerialUSB.println("Enabled Events:");
    SerialUSB.print("    CPU Reset: "); SerialUSB.println(enabledEventsValues.CPUReset);
    SerialUSB.print("    Error: "); SerialUSB.println(enabledEventsValues.Error);
    SerialUSB.print("    Quaternion Result: "); SerialUSB.println(enabledEventsValues.QuaternionResult);
    SerialUSB.print("    Mag Result: "); SerialUSB.println(enabledEventsValues.MagResult);
    SerialUSB.print("    Accel Result: "); SerialUSB.println(enabledEventsValues.AccelResult);
    SerialUSB.print("    Gyro Result: "); SerialUSB.println(enabledEventsValues.GyroResult);
    SerialUSB.print("    ReservedA: "); SerialUSB.println(enabledEventsValues.ReservedA);
    SerialUSB.print("    ReservedB: "); SerialUSB.println(enabledEventsValues.ReservedB);
}


// NOT WORKING????????
bool NaviguiderCompass::enableRawSensors(bool enableMag, bool enableAccel, bool enableGyro) {
    uint8_t value = 0x00;

    // Set bits for enabling raw sensors
    if (enableMag) value |= 0x04;    // Enable Magnetometer
    if (enableAccel) value |= 0x01; // Enable Accelerometer
    if (enableGyro) value |= 0x02;  // Enable Gyroscope

    uint8_t paramPage = PARAM_PAGE_WARM_START; // Replace with your defined warm start page register
    uint8_t registerAddress = PARAM_LOAD_REG; // Replace with your parameter load register address

    // Select the parameter page
    if (!m_i2c_dev->write(&paramPage, 1)) {
#ifdef DEBUG
        SerialUSB.println("Failed to set parameter page!");
#endif
        return false;
    }

    // Write the parameter value (value) to the parameter load register
    if (!m_i2c_dev->write_then_read(&registerAddress, 1, &value, 1)) {
#ifdef DEBUG
        SerialUSB.println("Failed to write raw sensor enable values!");
#endif
        return false;
    }

    // Commit the change
    uint8_t paramCommit = 0x80 | 127; // 0x80 sets the commit bit, 127 is the parameter ID
    if (!m_i2c_dev->write(&paramCommit, 1)) {
#ifdef DEBUG
        SerialUSB.println("Failed to commit parameter changes!");
#endif
        return false;
    }

#ifdef DEBUG
    SerialUSB.println("Raw sensors enabled successfully!");
#endif

    return true;
}