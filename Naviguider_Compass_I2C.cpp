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
PhysicalSensorStatusStruct NaviguiderCompass::PhysicalSensorStatus;
NaviguiderSensorNamesStruct NaviguiderCompass::NaviguiderSensorNames = {};


// Constructor with host interrupt pin
NaviguiderCompass::NaviguiderCompass(int hostInterruptPin)
    : _hostInterruptPin(hostInterruptPin) {
    // Set the host interrupt pin as input
    pinMode(_hostInterruptPin, INPUT);
    delay(500);  // Wait for sensor startup
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
	
	
	
	// Start the CPU
	startCPU();
	
	#ifdef DEBUG
	printNaviguiderStatusRegister();
	printNaviguiderSensorStatus();
	printPhysicalSensorStatus();
	printPhysicalSensorInformation();
    printSensorInformation();
	#endif
	
	
	
	// Set the MAG, ACCEL, and GYRO Sensor Rates
	////setSensorRates();
	
	////querySensorsPresent();
	
	// Configure the Algorithm Control Register
	////configAlgorithmControlRegister();
	
	// Read the 
	////algorithmControlValues = getAlgorithmControlValue();
	
	#ifdef DEBUG
	////printAlgorithmControlValues();
	#endif
	
	// Enable events
	// Example: Enable quaternion, error, and CPU reset events
    ////configureEnabledEventsRegister();
	
	// Read the enabled events register
	////enabledEventsValues = readEnabledEventsRegister();
	
	#ifdef DEBUG
	////printEnabledEventsValues();
	#endif
	
	
	
	// Check for errors
	#ifdef DEBUG
	////SerialUSB.print("Errors: ");
	////SerialUSB.println(checkForErrors());
	#endif
	
    return returnVal; // Initialization succeeded
}


/**
 * @brief Converts a given byte array into a human-readable binary string.
 * 
 * @param data Pointer to the byte array.
 * @param numBytes Number of bytes in the array.
 * @param outputStr Pointer to a character array where the binary string will be stored.
 *                  The caller must ensure it is at least `numBytes * 8 + 1` in size.
 * @return Pointer to the outputStr containing the binary representation.
 */
char* NaviguiderCompass::getBinaryStringFromBytes(const void* data, uint8_t numBytes, char* outputStr) {
    const uint8_t* byteArray = (const uint8_t*)data;
    
    for (uint8_t i = 0; i < numBytes; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            outputStr[i * 8 + (7 - j)] = (byteArray[(numBytes - 1) - i] & (1 << j)) ? '1' : '0';
        }
    }
    
    outputStr[numBytes * 8] = '\0'; // Null-terminate the string
    return outputStr;
}


/**
 * @brief Display the status registers of the Naviguider Compass.
 *
 * This function reads multiple status-related registers and prints their values 
 * in both decimal and binary formats for debugging.
 */
void NaviguiderCompass::printNaviguiderStatusRegister()
{
    uint8_t buf[4];    // Buffer to hold register values
    char str[17];      // String buffer for binary representation
    uint16_t bytesAvailable; // Holds the number of bytes remaining

    SerialUSB.println("\n------------ Naviguider Status Register -----------\n");

    // Read and display Host Status, Interrupt Status, and Chip Status
    uint8_t reg = NAVIGUIDER_REG_HOST_STATUS;
    if (!m_i2c_dev->write_then_read(&reg, 1, buf, 3, true)) {
        SerialUSB.println("Failed to read Host Status registers");
        return;
    }

    SerialUSB.print("Host Status:       ");
    SerialUSB.print(buf[0], HEX);
    SerialUSB.print(", ");
    SerialUSB.println(getBinaryStringFromBytes(buf, 1, str));

    SerialUSB.print("Interrupt Status:  ");
    SerialUSB.print(buf[1], HEX);
    SerialUSB.print(", ");
    SerialUSB.println(getBinaryStringFromBytes(buf + 1, 1, str));

    SerialUSB.print("Chip Status:       ");
    SerialUSB.print(buf[2], HEX);
    SerialUSB.print(", ");
    SerialUSB.println(getBinaryStringFromBytes(buf + 2, 1, str));

    // Read and display Error Register, Interrupt State, Debug Value, and Debug State
    reg = NAVIGUIDER_REG_ERROR;
    if (!m_i2c_dev->write_then_read(&reg, 1, buf, 4, true)) {
        SerialUSB.println("Failed to read Error Register");
        return;
    }

    SerialUSB.print("Error Register:    ");
    SerialUSB.print(buf[0], HEX);
    SerialUSB.print(", ");
    SerialUSB.println(getBinaryStringFromBytes(buf, 1, str));

    SerialUSB.print("Interrupt State:   ");
    SerialUSB.print(buf[1], HEX);
    SerialUSB.print(", ");
    SerialUSB.println(getBinaryStringFromBytes(buf + 1, 1, str));

    SerialUSB.print("Debug Value:       ");
    SerialUSB.print(buf[2], HEX);
    SerialUSB.print(", ");
    SerialUSB.println(getBinaryStringFromBytes(buf + 2, 1, str));

    SerialUSB.print("Debug State:       ");
    SerialUSB.print(buf[3], HEX);
    SerialUSB.print(", ");
    SerialUSB.println(getBinaryStringFromBytes(buf + 3, 1, str));

    // Read and display Bytes Remaining (16-bit value)
    reg = NAVIGUIDER_REG_BYTES_REMAINING_LSB;
    if (!m_i2c_dev->write_then_read(&reg, 1, (uint8_t*)&bytesAvailable, 2, true)) {
        SerialUSB.println("Failed to read Bytes Remaining register");
        return;
    }

    SerialUSB.print("Bytes Remaining:   ");
    SerialUSB.print(bytesAvailable);
    SerialUSB.print(", ");
    SerialUSB.println(getBinaryStringFromBytes((uint8_t*)&bytesAvailable, 2, str));

    SerialUSB.println();
}


/**
 * @brief Displays the status of all available sensors in the Naviguider system.
 *
 * This function queries the system parameter page for sensor status banks and
 * prints the information in a tabular format.
 */
void NaviguiderCompass::printNaviguiderSensorStatus()
{
    SerialUSB.println("+--------------------test1----------------------------+");

    SensorStatusStruct currentSensorStatus[32];  // Array to hold sensor statuses
    uint8_t sensorStatusBuffer[sizeof(currentSensorStatus)];  // Intermediate byte buffer
    ParameterInformation parameters[] =  { NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_0, 32 } ;

    // Read sensor status for sensors 1-32
    if (!readParameter(NAVIGUIDER_PARAMETER_PAGE_SYSTEM, parameters, 1, sensorStatusBuffer)) {
        SerialUSB.println("Failed to read sensor status (Bank 0)");
        return;
    }

    // Copy buffer to properly aligned struct
    memcpy(currentSensorStatus, sensorStatusBuffer, sizeof(currentSensorStatus));

    SerialUSB.println("+------------------------------------------------------------+");
    SerialUSB.println("|            NAVIGUIDER SENSOR STATUS                        |");
    SerialUSB.println("+-----+-----------+------+--------+-----------+------+-------+");
    SerialUSB.println("| ID  | Data      | I2C  | DEVICE | Transient | Data | Power |");
    SerialUSB.println("|     | Available | NACK | ID ERR | Error     | Lost | Mode  |");
    SerialUSB.println("+-----+-----------+------+--------+-----------+------+-------+");

    // Display sensor status for sensors 1-32
    for (uint8_t i = 0; i < 32; i++) {
        printSensorStatusRow(i + 1, &currentSensorStatus[i]);
    }


	uint8_t sensorStatusBuffer2[sizeof(currentSensorStatus)];
    // Read sensor status for sensors 65-80
    parameters[0].ParameterNumber = NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_0 + 4;
    ////parameters[0].ParameterNumber = NAVIGUIDER_SYSTEM_PARAMETER_SENSOR_STATUS_BANK_1;//this was a test
    if (!readParameter(NAVIGUIDER_PARAMETER_PAGE_SYSTEM, parameters, 1, sensorStatusBuffer2)) {
        SerialUSB.println("Failed to read sensor status (Bank 1)");
        return;
    }

    // Copy buffer to properly aligned struct
    memcpy(currentSensorStatus, sensorStatusBuffer2, sizeof(currentSensorStatus));

    // Display sensor status for sensors 65-80
    for (uint8_t i = 0; i < 16; i++) {
        printSensorStatusRow(i + 65, &currentSensorStatus[i]);
    }

    SerialUSB.println("+-----+-----------+------+--------+-----------+------+-------+");
}




/**
 * @brief Displays the physical sensor status, including sample rate, dynamic range, and status flags.
 *
 * This function retrieves and prints the status of the physical sensors (Accelerometer, Gyroscope, Magnetometer).
 * The output is formatted in a structured table for readability.
 */
void NaviguiderCompass::printPhysicalSensorStatus()
{
    // Retrieve physical sensor status
    readPhysicalSensorStatus();

    // Print table header
    SerialUSB.println("+----------------------------------------------------------------------------------------+");
    SerialUSB.println("|                       NAVIGUIDER PHYSICAL SENSOR STATUS                                |");
    SerialUSB.println("+--------+--------+---------+-----+-----------+------+--------+-----------+------+-------+");
    SerialUSB.println("| Sensor | Sample | Dynamic | ID  | Data      | I2C  | DEVICE | Transient | Data | Power |");
    SerialUSB.println("|        | Rate   | Range   |     | Available | NACK | ID ERR | Error     | Lost | Mode  |");
    SerialUSB.println("+--------+--------+---------+-----+-----------+------+--------+-----------+------+-------+");

    // Print sensor details using formatted output
    char buffer[128];

    // Accelerometer
    sprintf(buffer, "| Accel  | %6u | %7u |", 
            PhysicalSensorStatus.Accelerometer.SampleRate, 
            PhysicalSensorStatus.Accelerometer.DynamicRange);
    SerialUSB.print(buffer);
    printSensorStatusRow(1, &PhysicalSensorStatus.Accelerometer.Status);

    // Gyroscope
    sprintf(buffer, "| Gyro   | %6u | %7u |", 
            PhysicalSensorStatus.Gyroscope.SampleRate, 
            PhysicalSensorStatus.Gyroscope.DynamicRange);
    SerialUSB.print(buffer);
    printSensorStatusRow(2, &PhysicalSensorStatus.Gyroscope.Status);

    // Magnetometer
    sprintf(buffer, "| Mag    | %6u | %7u |", 
            PhysicalSensorStatus.Magnetometer.SampleRate, 
            PhysicalSensorStatus.Magnetometer.DynamicRange);
    SerialUSB.print(buffer);
    printSensorStatusRow(3, &PhysicalSensorStatus.Magnetometer.Status);

    // Print table footer
    SerialUSB.println("+--------+--------+---------+-----+-----------+------+--------+-----------+------+-------+");
}


/**
 * @brief Reads the physical sensor status from the Naviguider.
 *
 * This function queries the Naviguider to retrieve the current status, 
 * sample rate, and dynamic range of all physical sensors (accelerometer, 
 * gyroscope, and magnetometer) and stores the results in the 
 * `PhysicalSensorStatusStruct` global structure.
 */
void NaviguiderCompass::readPhysicalSensorStatus()
{
    uint8_t buffer[15]; // Buffer to store the retrieved parameter data

    // Define the parameter information structure
    ParameterInformation param[] = {{ NAVIGUIDER_SYSTEM_PARAMETER_PHYSICAL_SENSOR_STATUS, sizeof(buffer) }};

    // Attempt to read the physical sensor status parameter
    if (!readParameter(
            NAVIGUIDER_PARAMETER_PAGE_SYSTEM, // Page number
            param, // Pointer to a single ParameterInformation structure
            1, // Number of parameters (only one in this case)
            buffer)) // Output buffer
    {
        SerialUSB.println("Failed to read physical sensor status.");
        return;
    }

    // Copy the buffer data into the PhysicalSensorStatusStruct
    memcpy(&PhysicalSensorStatus, buffer, sizeof(buffer));
}



/**
 * @brief Displays the status information for a given sensor.
 *
 * This function prints the status of an individual sensor, including its ID,
 * data availability, communication errors, transient errors, and power mode.
 *
 * @param sensorId The ID of the sensor being displayed.
 * @param status A pointer to the SensorStatusStruct containing the sensor's status.
 */
void NaviguiderCompass::printSensorStatusRow(uint8_t sensorId, const SensorStatusStruct *status)
{
     // Ensuring consistent column width by adding leading spaces where necessary
    SerialUSB.print("| ");
    if (sensorId < 10) SerialUSB.print(" ");  // Add extra space for single-digit IDs
    SerialUSB.print(sensorId);
    SerialUSB.print("  |     ");
    SerialUSB.print(status->IsDataAvailable);
    SerialUSB.print("     |  ");
    SerialUSB.print(status->IsI2CNack);
    SerialUSB.print("   |   ");
    SerialUSB.print(status->IsDeviceIDError);
    SerialUSB.print("    |     ");
    SerialUSB.print(status->IsTransientError);
    SerialUSB.print("     |  ");
    SerialUSB.print(status->IsDataLost);
    SerialUSB.print("   |   ");
    SerialUSB.print(status->PowerMode);
    SerialUSB.println("  |");
}


/**
 * @brief Prints detailed information about each physical sensor present in the system.
 *
 * This function queries the physical sensors, retrieves their properties, and prints 
 * them in a formatted table including sensor type, driver ID, version, power consumption, 
 * dynamic range, sample rate, and number of axes.
 */
void NaviguiderCompass::printPhysicalSensorInformation()
{
    // Variable to store the bitmap of present physical sensors (64-bit)
    uint64_t physicalSensorPresent = 0;

    // Define the parameter structure for querying sensor presence (8-byte bitmap)
    ParameterInformation paramPresence = { 
        NAVIGUIDER_SYSTEM_PARAMETER_PHYSICAL_SENSORS_PRESENT, // Parameter number for sensor presence bitmap
        4
    };

    // Read the bitmap of physical sensors present FAILS HERE FOR SOME REASNO????
    if (!readParameter(
            NAVIGUIDER_PARAMETER_PAGE_SYSTEM, 
            &paramPresence, 
            1, 
            (uint8_t*)&physicalSensorPresent)) 
    {
        SerialUSB.println("Failed to read physical sensor presence bitmap.");
        return;
    }


    // Print table header
    SerialUSB.println("\n+----------------------------------+--------+---------+-------+---------+------------+------+");
    SerialUSB.println("| Sensor                           | Driver | Driver  | Power | Current | Current    | Num  |");
    SerialUSB.println("|                                  | ID     | Version |       | Range   | Rate       | Axes |");
    SerialUSB.println("+----------------------------------+--------+---------+-------+---------+------------+------+");

    // Iterate through all possible physical sensors (up to 64)
    for (uint32_t sensorId = 0; sensorId < 64; sensorId++)
    {
        // Check if the sensor is present in the bitmap
        if (physicalSensorPresent & (1LL << sensorId))
        {
            // Structure to store physical sensor information
            PhysicalSensorInformationStruct sensorInfo;
            paramPresence.DataSize = 16;
            paramPresence.ParameterNumber = sensorId + 32;
           


            
            /*
            // Read sensor information
            if (!readParameter(
                    NAVIGUIDER_PARAMETER_PAGE_SYSTEM, 
                    &paramPresence,
                    1, 
                    (uint8_t*)&sensorInfo)) 
            {
                SerialUSB.print("Failed to read information for sensor ID: ");
                SerialUSB.println(sensorId);
                continue;
            }
            */
            uint8_t buffer[16];

            if (!readParameter(
                NAVIGUIDER_PARAMETER_PAGE_SYSTEM,
                &paramPresence,
                1,
                buffer))
            {
                SerialUSB.print("Failed to read information for sensor ID: ");
                SerialUSB.println(sensorId);
                continue;
            }


            memcpy(&sensorInfo, buffer, sizeof(buffer));
            
            
            
            
            // Print sensor information in a formatted row
            SerialUSB.print("| ");

            
            SerialUSB.print(getSensorName(sensorInfo.SensorType));


           
            // Pad the sensor name to align columns
            for (int i = strlen(getSensorName(sensorInfo.SensorType)); i < 32; i++) SerialUSB.print(" ");
            SerialUSB.print(" |   ");

            SerialUSB.print(sensorInfo.DriverID);
            SerialUSB.print(" |    ");

            SerialUSB.print(sensorInfo.DriverVersion);
            SerialUSB.print(" |  ");

            SerialUSB.print(sensorInfo.CurrentConsumption);
            SerialUSB.print(" |    ");

            SerialUSB.print(sensorInfo.DynamicRange);
            SerialUSB.print(" |       ");

            SerialUSB.print(sensorInfo.SampleRate);
            SerialUSB.print(" |  ");

            SerialUSB.print(sensorInfo.NumAxes);
            SerialUSB.println(" |");
            
            
        }
    }

    // Print table footer
    SerialUSB.println("+----------------------------------+--------+---------+-------+---------+------------+------+");
    
}




const char* NaviguiderCompass::getSensorName(uint8_t sensorId)
{
    switch (sensorId) {
    case NAVIGUIDER_SENSOR_TYPE_NA:                          return "na";
    case NAVIGUIDER_SENSOR_TYPE_ACCELEROMETER:               return "accelerometer";
    case NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD:              return "magnetic field";
    case NAVIGUIDER_SENSOR_TYPE_ORIENTATION:                 return "orientation";
    case NAVIGUIDER_SENSOR_TYPE_GYROSCOPE:                   return "gyroscope";
    case NAVIGUIDER_SENSOR_TYPE_LIGHT:                       return "light";
    case NAVIGUIDER_SENSOR_TYPE_PRESSURE:                    return "pressure";
    case NAVIGUIDER_SENSOR_TYPE_TEMPERATURE:                 return "temperature";
    case NAVIGUIDER_SENSOR_TYPE_PROXIMITY:                   return "proximity";
    case NAVIGUIDER_SENSOR_TYPE_GRAVITY:                     return "gravity";
    case NAVIGUIDER_SENSOR_TYPE_LINEAR_ACCELERATION:         return "linear acceleration";
    case NAVIGUIDER_SENSOR_TYPE_ROTATION_VECTOR:             return "rotation vector";
    case NAVIGUIDER_SENSOR_TYPE_RELATIVE_HUMIDITY:           return "relative humidity";
    case NAVIGUIDER_SENSOR_TYPE_AMBIENT_TEMPERATURE:         return "ambient temperature";
    case NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:        return "magnetic field uncalibrated";
    case NAVIGUIDER_SENSOR_TYPE_GAME_ROTATION_VECTOR:        return "game rotation vector";
    case NAVIGUIDER_SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:             return "gyroscope uncalibrated";
    case NAVIGUIDER_SENSOR_TYPE_SIGNIFICANT_MOTION:          return "significant motion";
    case NAVIGUIDER_SENSOR_TYPE_STEP_DETECTOR:               return "step detector";
    case NAVIGUIDER_SENSOR_TYPE_STEP_COUNTER:                return "step counter";
    case NAVIGUIDER_SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:    return "geomagnetic rotation vector";
    case NAVIGUIDER_SENSOR_TYPE_HEART_RATE:       return "heart rate -OR- car detector";
    case NAVIGUIDER_SENSOR_TYPE_TILT_DETECTOR:               return "tilt detector";
    case NAVIGUIDER_SENSOR_TYPE_WAKE_GESTURE:                return "wake gesture";
    case NAVIGUIDER_SENSOR_TYPE_GLANCE_GESTURE:              return "glance gesture";
    case NAVIGUIDER_SENSOR_TYPE_PICK_UP_GESTURE:             return "pick up gesture";
    case NAVIGUIDER_SENSOR_TYPE_RAW_ACCEL:                   return "raw accel";
    case NAVIGUIDER_SENSOR_TYPE_RAW_MAG:                     return "raw mag";
    case NAVIGUIDER_SENSOR_TYPE_RAW_GYRO:                    return "raw gyro";
    case NAVIGUIDER_SENSOR_TYPE_ACTIVITY:                    return "activity";
    case NAVIGUIDER_SENSOR_TYPE_CAR_MAG_DATA:         return "car detect mag data (uT)";

        // Wake versions of sensors
    case NAVIGUIDER_SENSOR_TYPE_ACCELEROMETER_WAKE:          return "accelerometer wake";
    case NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD_WAKE:         return "magnetic field wake";
    case NAVIGUIDER_SENSOR_TYPE_ORIENTATION_WAKE:            return "orientation wake";
    case NAVIGUIDER_SENSOR_TYPE_GYROSCOPE_WAKE:              return "gyroscope wake";
    case NAVIGUIDER_SENSOR_TYPE_LIGHT_WAKE:                  return "light wake";
    case NAVIGUIDER_SENSOR_TYPE_PRESSURE_WAKE:               return "pressure wake";
    case NAVIGUIDER_SENSOR_TYPE_TEMPERATURE_WAKE:            return "temperature wake";
    case NAVIGUIDER_SENSOR_TYPE_PROXIMITY_WAKE:              return "proximity wake";
    case NAVIGUIDER_SENSOR_TYPE_GRAVITY_WAKE:                return "gravity wake";
    case NAVIGUIDER_SENSOR_TYPE_LINEAR_ACCEL_WAKE:           return "linear acceleration wake";
    case NAVIGUIDER_SENSOR_TYPE_ROTATION_VECTOR_WAKE:        return "rotation vector wake";
    case NAVIGUIDER_SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE:      return "relative humidity wake";
    case NAVIGUIDER_SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE:    return "ambient temperature wake";
    case NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD_UNCAL_WAKE:   return "magnetic field uncalibrated wake";
    case NAVIGUIDER_SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE:   return "game rotation vector wake";
    case NAVIGUIDER_SENSOR_TYPE_GYROSCOPE_UNCAL_WAKE:        return "gyroscope uncalibrated wake";
    case NAVIGUIDER_SENSOR_TYPE_SIGNIFICANT_MOTION_WAKE:     return "significant motion wake";

    default:
        return "unknown sensor";
    }
}




/**
 * @brief Retrieves sensor configuration for all valid sensors.
 *
 * This function iterates through the sensor information array and, for each valid sensor,
 * reads its configuration parameters from the Naviguider.
 */
void NaviguiderCompass::getSensorConfiguration()
{
    // Loop through all sensors
    for (uint8_t i = 1; i < sizeof(sensorInformation) / sizeof(sensorInformation[0]); i++)
    { 
        if (sensorInformation[i].sensorId > 0)
        {
            // Define parameter structure for the current sensor
            ParameterInformation param = { sensorInformation[i].sensorId, sizeof(sensorConfiguration[i]) };

            // Read sensor configuration
            readParameter(
                NAVIGUIDER_PARAMETER_PAGE_SENSOR_CONFIG, // Page number
                &param, // Pointer to parameter information
                1, // Number of parameters (just one)
                (uint8_t*)&sensorConfiguration[i] // Output buffer
            );
        }
    }
}



/**
 * @brief Retrieves information for all sensors from the Naviguider.
 *
 * This function reads sensor information into the `sensorInformation` structure,
 * then extracts and stores the maximum data rates for the magnetometer, accelerometer, and gyroscope.
 */
void NaviguiderCompass::getSensorInformation()
{
    
    /*
    // Read sensor information from the Naviguider
    if (!readParameter(
            NAVIGUIDER_PARAMETER_PAGE_SENSOR_INFO, // Page number
            sensorInfoParamList, // Parameter list
            sizeof(sensorInfoParamList) / sizeof(sensorInfoParamList[0]), // Number of parameters
            (uint8_t*)sensorInformation)) // Output buffer
    {
        SerialUSB.println("Error: Failed to read sensor information.");
        return;
    }*/


    // loop through every possible sensor to see if we can get info from it.
    // Define the parameter structure for querying sensor presence (8-byte bitmap)
    uint8_t numSensors = 33;
    ParameterInformation paramPresence[numSensors];  // Create an array of 128 entries

    // Populate array using a loop
    for (uint8_t i = 0; i < numSensors; i++) {
        paramPresence[i].ParameterNumber = i;  // Assign increasing numbers
        paramPresence[i].DataSize = 16;        // Constant data size
    }

    // Temporary buffer to store sensor information before copying

    SerialUSB.println("Reading Sensor Information");


    
    //uint8_t sensorInfoBuffer[sizeof(sensorInformation)];
    uint8_t sensorInfoBuffer[numSensors * 16];
    
    if (!readParameter(
        NAVIGUIDER_PARAMETER_PAGE_SENSOR_INFO,
        paramPresence, // Parameter list
        numSensors, // Number of parameters
        sensorInfoBuffer))
    {
        SerialUSB.println("Error: Failed to read sensor information.");
        return;
    }

    for (int j = 0; j < sizeof(sensorInfoBuffer) - 1; j++) {
        SerialUSB.print("sensorInfoBuffer: "); SerialUSB.println(sensorInfoBuffer[j], HEX);

    }

    

    

    SerialUSB.println("DONE READING Reading Sensor Information");


    // Copy the data from the buffer into the sensorInformation structure
    memcpy(sensorInformation, sensorInfoBuffer, sizeof(sensorInfoBuffer));
    
    SerialUSB.println("memcopy done");

    // Mark that we now have valid sensor information
    haveSensorInformation = true;

    // Extract maximum data rates for individual sensors
    magMaxRate = sensorInformation[NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD].maxRate;
    accelMaxRate = sensorInformation[NAVIGUIDER_SENSOR_TYPE_ACCELEROMETER].maxRate;
    gyroMaxRate = sensorInformation[NAVIGUIDER_SENSOR_TYPE_GYROSCOPE].maxRate;
    
}


void NaviguiderCompass::printSensorInformation()
{

    

    // Ensure sensor information is available before proceeding
    if (!haveSensorInformation) {
        getSensorInformation();
    }

    

    // Print table header
    SerialUSB.println("+------------------------------------------------------------------------------------------+");
    SerialUSB.println("|                       NaviGuider Sensor Information                                      |");
    SerialUSB.println("+-----+----------------------------------+---------+-------+-------+-----+----------+------+");
    SerialUSB.println("| ID  | Sensor                           | Driver  | Power | Range | Res | Rate     | Size |");
    SerialUSB.println("+-----+----------------------------------+---------+-------+-------+-----+----------+------+");

    

    // Iterate over available sensor information
    for (uint8_t i = 0; i < (sizeof(sensorInfoParamList) / sizeof(sensorInfoParamList[0])); i++)
    {
        if (sensorInformation[i].sensorId > 0)
        {
            SerialUSB.print("| ");
            SerialUSB.print(i);
            SerialUSB.print("  | ");

            // Print sensor name and align to 32 characters
            SerialUSB.print(getSensorName(sensorInformation[i].sensorId));
            uint8_t nameLength = strlen(getSensorName(sensorInformation[i].sensorId));
            for (uint8_t j = nameLength; j < 32; j++) {
                SerialUSB.print(" ");
            }
            SerialUSB.print(" | ");

            // Print driver version as "X.YYY" format with leading zeros
            SerialUSB.print(sensorInformation[i].driverId);
            SerialUSB.print(".");
            if (sensorInformation[i].driverVersion < 10) {
                SerialUSB.print("00");
            }
            else if (sensorInformation[i].driverVersion < 100) {
                SerialUSB.print("0");
            }
            SerialUSB.print(sensorInformation[i].driverVersion);
            SerialUSB.print(" | ");

            // Align numerical values properly
            char buffer[32];  // Buffer to ensure consistent spacing

            sprintf(buffer, "%5u | ", sensorInformation[i].power);
            SerialUSB.print(buffer);

            sprintf(buffer, "%5u | ", sensorInformation[i].maxRange);
            SerialUSB.print(buffer);

            sprintf(buffer, "%3u | ", sensorInformation[i].resolution);
            SerialUSB.print(buffer);

            sprintf(buffer, "%3u-%-4u | ", sensorInformation[i].minRate, sensorInformation[i].maxRate);
            SerialUSB.print(buffer);

            sprintf(buffer, "%4u |", sensorInformation[i].eventSize);
            SerialUSB.println(buffer);
        }
    }


    /*
    char buffer[128];  // Buffer for formatted output
    // Iterate over available sensor information
    for (uint8_t i = 0; i < (sizeof(sensorInfoParamList) / sizeof(sensorInfoParamList[0])); i++)
    {
        if (sensorInformation[i].sensorId > 0)
        {
            // Get sensor name and pad to 32 characters
            char sensorName[33];  // 32 characters + null terminator
            snprintf(sensorName, sizeof(sensorName), "%-32s", getSensorName(sensorInformation[i].sensorId));

            // Format and print the row
            sprintf(buffer, "| %3u | %s | %3u.%-3u | %5u | %5u | %3u | %3u-%-4u | %4u |",
                i,
                sensorName,
                sensorInformation[i].driverId,
                sensorInformation[i].driverVersion,
                sensorInformation[i].power,
                sensorInformation[i].maxRange,
                sensorInformation[i].resolution,
                sensorInformation[i].minRate,
                sensorInformation[i].maxRate,
                sensorInformation[i].eventSize);

            SerialUSB.println(buffer);
        }
    }

    */

    // Print table footer
    SerialUSB.println("+-----+----------------------------------+---------+-------+-------+-----+----------+------+");

    
}



/*
void NaviguiderCompass::printSensorConfiguration()
{
    // Ensure sensor information is available before proceeding
    if (!haveSensorInformation) {
        getSensorInformation();
    }

    // Retrieve sensor configuration
    getSensorConfiguration();

    // Print table header
    SerialUSB.println("+-------------------------------------------------------------------------+");
    SerialUSB.println("|                          Sensor Configuration                           |");
    SerialUSB.println("+----------------------------------+-------+-------+-------------+--------+");
    SerialUSB.println("| Sensor                           | Rate  | Delay | Sensitivity | Range  |");
    SerialUSB.println("+----------------------------------+-------+-------+-------------+--------+");

    // Iterate over available sensor information
    for (uint8_t i = 0; i < (sizeof(sensorInformation) / sizeof(sensorInformation[0])); i++)
    {
        if (sensorInformation[i].sensorId > 0)
        {
            SerialUSB.print("| ");
            SerialUSB.print(getSensorName(sensorInformation[i].sensorId));

            // Align sensor name to 32 characters
            uint8_t nameLength = strlen(getSensorName(sensorInformation[i].sensorId));
            for (uint8_t j = nameLength; j < 32; j++) {
                SerialUSB.print(" ");
            }

            SerialUSB.print(" | ");
            SerialUSB.print(sensorConfiguration[i].sampleRate);
            SerialUSB.print(" | ");
            SerialUSB.print(sensorConfiguration[i].maxReportLatency);
            SerialUSB.print(" | ");
            SerialUSB.print(sensorConfiguration[i].changeSensitivity);
            SerialUSB.print(" | ");
            SerialUSB.print(sensorConfiguration[i].dynamicRange);
            SerialUSB.println(" |");
        }
    }

    // Print table footer
    SerialUSB.println("+----------------------------------+-------+-------+-------------+--------+");
}

*/





/*
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
*/


/*
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

*/

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
	//SerialUSB.println("Interrupt Occurred");
	#endif
}


/*
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
*/


/**
 * @brief Writes a parameter to the Naviguider sensor.
 * 
 * @param page Parameter page number (0x54 to select page)
 * @param paramNumber Parameter number to write
 * @param data Pointer to the data buffer
 * @param dataSize Size of the data (max 4 bytes)
 * @return true if successful, false if failed
 */
bool NaviguiderCompass::writeParameter(uint8_t page, uint8_t paramNumber, uint8_t* data, uint8_t dataSize) {
    uint8_t pageSelectValue = page | (dataSize << 4);
    uint8_t reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
    if (!m_i2c_dev->write(&pageSelectValue, 1, true, &reg, 1)) {
        SerialUSB.print("writeParameter: Failed to set parameter page (0x54) to: 0x");
        SerialUSB.println(page, HEX);
        return false;
    }

    // Load the Parameter Data into registers 0x5C-0x5F
    for (uint8_t i = 0; i < dataSize && i < 4; i++) {
        uint8_t reg = NAVIGUIDER_REG_LOAD_PARAM_BYTE_0 + i;
        if (!m_i2c_dev->write(&data[i], 1, true, &reg, 1)) {
			SerialUSB.println("writeParameter: Failed to Load the Parameter Data into registers 0x5C-0x5F");
            return false;
        }
    }

    // Send Parameter Request (Register: 0x64, Data: paramNumber | 0x80)
    uint8_t paramReq = paramNumber | 0x80;
    reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
    if (!m_i2c_dev->write(&paramReq, 1, true, &reg, 1)) {
		SerialUSB.println("writeParameter: Failed to Send Parameter Request (Register: 0x64, Data: paramNumber | 0x80");
        return false;
    }

    // Wait for Acknowledgment
    uint8_t ack = 0;
    reg = NAVIGUIDER_REG_PARAMETER_ACKNOWLEDGE;
    do {
        if (!m_i2c_dev->write_then_read(&reg, 1, &ack, 1)) { return false; }
        if (ack == 0x80) {  
            uint8_t resetVal = 0;
			uint8_t reg_req = NAVIGUIDER_REG_PARAMETER_REQUEST;
			uint8_t reg_page_select = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
            m_i2c_dev->write(&resetVal, 1, true, &reg_req);
            m_i2c_dev->write(&resetVal, 1, true, &reg_page_select);
			SerialUSB.println("writeParameter: Failed to Wait for Acknowledgment");
            return false;
        }
    } while (ack != paramReq);

    return true;
}



/**
 * @brief Reads one or more parameters from the Naviguider sensor.
 *
 * This function reads parameter values from the sensor by setting the page, requesting the
 * parameter, waiting for acknowledgment, and retrieving the saved parameter data.
 *
 * @param page The parameter page number to select.
 * @param paramList Pointer to an array of ParameterInformation structures specifying the parameters to read.
 * @param numParams The number of parameters to read.
 * @param values Pointer to the buffer where the retrieved parameter values will be stored.
 * @return true if successful, false otherwise.
 */
bool NaviguiderCompass::readParameter(uint8_t page, const ParameterInformation* paramList, uint8_t numParams, uint8_t* values)
{
    // Debugging: Print all arguments passed into readParameter
    SerialUSB.print("readParameter called with page: 0x");
    SerialUSB.print(page, HEX);
    SerialUSB.print(", numParams: ");
    SerialUSB.println(numParams);

    // Print details of each parameter in paramList
    for (uint8_t i = 0; i < numParams; i++) {
        SerialUSB.print("  Param ");
        SerialUSB.print(i);
        SerialUSB.print(": ParameterNumber = 0x");
        SerialUSB.print(paramList[i].ParameterNumber, HEX);
        SerialUSB.print(", DataSize = ");
        SerialUSB.println(paramList[i].DataSize);
    }

    // Print output buffer address
    SerialUSB.print("  Output buffer address: 0x");
    SerialUSB.println((uintptr_t)values, HEX);


    uint8_t pageSelectValue, paramAck;
    uint16_t valueIndex = 0;

    
    for (uint8_t i = 0; i < numParams; i++)
    {
		
		
		// Step 1: Write the Page Number to Parameter Page Select Register
		pageSelectValue = page;
		uint8_t reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
		if (!m_i2c_dev->write(&pageSelectValue, 1, true, &reg, 1))
		{
			SerialUSB.print("readParameter: Failed to set parameter page (0x54) to: 0x");
			SerialUSB.println(pageSelectValue, HEX);
			return false;
		}
		SerialUSB.print("readParameter: Successfully set parameter page (0x54) to: 0x");
		SerialUSB.println(pageSelectValue, HEX);



        // Step 2: Write the Parameter Number to Parameter Request Register
        reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
		
		SerialUSB.print("writing to register (0x64) a value of: 0x");
		SerialUSB.println(paramList[i].ParameterNumber, HEX);
		
        if (!m_i2c_dev->write(&paramList[i].ParameterNumber, 1, false, &reg, 1))
        {
            SerialUSB.println("readParameter: Failed to send Parameter Request");
            return false;
        }

        // Poll Parameter Acknowledge Register (Use do-while like the original)
		uint8_t ack = 0;
		reg = NAVIGUIDER_REG_PARAMETER_ACKNOWLEDGE;
		do {
			if (!m_i2c_dev->write_then_read(&reg, 1, &ack, 1))
			{
				SerialUSB.println("I2C read failed while polling ACK (0x3A)");
				return false;
			}

			if (ack == 0x80)
			{
				SerialUSB.println("Error: Parameter request not supported (ACK = 0x80)");

				// Reset registers
				uint8_t resetVal = 0;
				reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
				m_i2c_dev->write(&resetVal, 1, true, &reg, 1);
				
				reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
				m_i2c_dev->write(&resetVal, 1, true, &reg, 1);

				return false;
			}
		} while (ack != paramList[i].ParameterNumber);

        SerialUSB.print("Parameter Acknowledged: 0x");
        SerialUSB.println(ack, HEX);

        SerialUSB.print("Reading Register NAVIGUIDER_REG_SAVED_PARAM_BYTE_0 (0x3B), into values for return, NUMBYTES :");
        SerialUSB.println(paramList[i].DataSize);


        // Step 4: Read the Parameter Data from Saved Parameter Bytes, i think there's an issue here reading data
        reg = NAVIGUIDER_REG_SAVED_PARAM_BYTE_0;
        if (!m_i2c_dev->write_then_read(&reg, 1, &values[valueIndex], paramList[i].DataSize))
        {
            SerialUSB.println("readParameter: Failed to read Parameter Data");
            return false;
        }

        valueIndex += paramList[i].DataSize;
    }


    /*

    SerialUSB.println("// Step 6: End the parameter transfer procedure by writing 0 to the Parameter Request register Writing to: NAVIGUIDER_REG_PARAMETER_REQUEST (0x64)");
 

    // Step 6: End the parameter transfer procedure by writing 0 to the Parameter Request register
    uint8_t resetVal = 0;
    uint8_t reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
    if (!m_i2c_dev->write(&resetVal, 1, true, &reg, 1))
    {
        SerialUSB.println("Error: Failed to reset NAVIGUIDER_REG_PARAMETER_REQUEST");
        return false;
    }
	
	
    SerialUSB.println("// Step 7: End the parameter transfer procedure by writing 0 to the Parameter Request register Writing to: NAVIGUIDER_REG_PARAMETER_PAGE_SELECT (0x54)");


	// Step 7: End the parameter transfer procedure by writing 0 to the Parameter Request register
    resetVal = 0;
    reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
    if (!m_i2c_dev->write(&resetVal, 1, true, &reg, 1))
    {
        SerialUSB.println("Error: Failed to reset NAVIGUIDER_REG_PARAMETER_PAGE_SELECT");
        return false;
    }

    */
	

    return true;
}








/**
 * @brief Writes a parameter to the Naviguider sensor.
 * 
 * @param page Parameter page number (0x54 to select page)
 * @param paramNumber Parameter number to write
 * @param data Pointer to the data buffer
 * @param dataSize Size of the data (max 4 bytes)
 * @return true if successful, false if failed
 */
 /*
bool NaviguiderCompass::writeParameter(uint8_t page, uint8_t paramNumber, uint8_t* data, uint8_t dataSize) {
    // Set the Parameter Page (Register: 0x54, Data: page)
    uint8_t reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
    if (!m_i2c_dev->write(&page, 1, true, &reg, 1)) {
        SerialUSB.print("Failed to set parameter page (0x54) to: 0x");
        SerialUSB.println(page, HEX);
        return false;
    }

    // Load the Parameter Data into registers 0x5C-0x5F
    for (uint8_t i = 0; i < dataSize && i < 4; i++) {
        uint8_t reg = NAVIGUIDER_REG_LOAD_PARAM_BYTE_0 + i;
        if (!m_i2c_dev->write(&data[i], 1, true, &reg, 1)) {
            SerialUSB.print("Failed to write LoadParamByte ");
            SerialUSB.print(i);
            SerialUSB.print(" (Register: 0x");
            SerialUSB.print(reg, HEX);
            SerialUSB.print(") Data: 0x");
            SerialUSB.println(data[i], HEX);
            return false;
        }
    }

    // Send Parameter Request (Register: 0x64, Data: paramNumber | 0x80)
    uint8_t paramReq = paramNumber | 0x80; // Set MSB to indicate a 'Load' operation
    reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
    if (!m_i2c_dev->write(&paramReq, 1, true, &reg, 1)) {
        SerialUSB.print("Failed to send parameter request (0x64) Data: 0x");
        SerialUSB.println(paramReq, HEX);
        return false;
    }

    // Wait for Acknowledgment (Register: 0x3A)
    uint8_t ack = 0;
    reg = NAVIGUIDER_REG_PARAMETER_ACKNOWLEDGE;
    for (int i = 0; i < 100; i++) {  // Retry for up to 1 second
        if (!m_i2c_dev->write_then_read(&reg, 1, &ack, 1)) {
            SerialUSB.println("Failed to read Parameter Acknowledge (0x3A)");
            return false;
        }
        if (ack == paramReq) break; // Acknowledgment received
        delay(10);
    }

    if (ack != paramReq) {
        SerialUSB.println("Parameter Acknowledge timeout (0x3A)");
        return false;
    }

    SerialUSB.println("Parameter successfully written.");
    return true;
}
*/

/**
 * @brief Reads a parameter from the Naviguider sensor.
 * 
 * @param page Parameter page number (0x54 to select page)
 * @param paramNumber Parameter number to read
 * @param buffer Pointer to buffer where result will be stored
 * @param bufferSize Number of bytes to read (max 4)
 * @return true if successful, false if failed
 */
 /*
bool NaviguiderCompass::readParameter(uint8_t page, uint8_t paramNumber, uint8_t* buffer, uint8_t bufferSize) {
    // Set the Parameter Page (Register: 0x54, Data: page)
    uint8_t reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
    if (!m_i2c_dev->write(&page, 1, true, &reg, 1)) {
        SerialUSB.print("Failed to set parameter page "); SerialUSB.print(reg, HEX); SerialUSB.print("  to: 0x");
        SerialUSB.println(page, HEX);
        return false;
    }

    // Send Parameter Request (Register: 0x64, Data: paramNumber)
    uint8_t paramReq = paramNumber & 0x7F; // Clear MSB for 'Retrieve' operation
    reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
    if (!m_i2c_dev->write(&paramReq, 1, true, &reg, 1)) {
        SerialUSB.print("Failed to send parameter request (0x64) Data: 0x");
        SerialUSB.println(paramReq, HEX);
        return false;
    }

    // Wait for Acknowledgment (Register: 0x3A)
    uint8_t ack = 0;
    reg = NAVIGUIDER_REG_PARAMETER_ACKNOWLEDGE;
    for (int i = 0; i < 100; i++) {  // Retry for up to 1 second
        if (!m_i2c_dev->write_then_read(&reg, 1, &ack, 1)) {
            //SerialUSB.println("Failed to read Parameter Acknowledge (0x3A)");
            return false;
        }
        if (ack == paramReq){
			//SerialUSB.print("Param Ack Received: "); SerialUSB.println(ack, HEX);
			break; // Acknowledgment received
		}
        delay(10);
    }

    if (ack != paramReq) {
        //SerialUSB.println("Parameter Acknowledge timeout (0x3A)");
        return false;
    }

    // Read Parameter Data from registers 0x3B-0x3E
    for (uint8_t i = 0; i < bufferSize && i < 4; i++) {
        uint8_t reg = NAVIGUIDER_REG_SAVED_PARAM_BYTE_0 + i;
        if (!m_i2c_dev->write_then_read(&reg, 1, &buffer[i], 1)) {
            //SerialUSB.print("Failed to read RetrieveParamByte ");
            //SerialUSB.print(i);
            //SerialUSB.print(" (Register: 0x");
            //SerialUSB.print(reg, HEX);
            //SerialUSB.println(")");
            return false;
        }
    }

    //SerialUSB.println("Parameter successfully read.");
    return true;
}
*/



















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


/*
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
*/

/*
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
*/


/**
 * @brief Queries the physical sensors present on the Naviguider device.
 *        Reads the Physical Sensors Present bitmap from Parameter Page 1, Parameter 32.
 * @return SensorsPresentBitmapStruct struct with detected sensors set to true.
 */
 /*
SensorsPresentBitmapStruct NaviguiderCompass::querySensorsPresent() {
    SensorsPresentBitmapStruct sensors = {0}; // Initialize all fields to false

    uint8_t paramPage = NAVIGUIDER_PARAMETER_PAGE_SYSTEM;   // SYSTEM Parameter Page
    uint8_t paramNumber = NAVIGUIDER_SYSTEM_PARAMETER_PHYSICAL_SENSORS_PRESENT; // Parameter 32 - Physical Sensors Present
    uint8_t bitmap[8] = {0};    // 64-bit sensor presence bitmap

    // Read the sensor presence bitmap from the device
    if (!readParameter(paramPage, paramNumber, bitmap, sizeof(bitmap))) {
        #ifdef DEBUG
        SerialUSB.println("Failed to read Physical Sensors Present bitmap!");
        #endif
        return sensors; // Return default struct with all sensors set to false
    }

    // Parse bitmap to determine available sensors
    sensors.accelerometer                 = (bitmap[0] & (1 << 0)) != 0;  // Sensor ID 0x01
    sensors.magnetometer                  = (bitmap[0] & (1 << 1)) != 0;  // Sensor ID 0x02
    sensors.orientation                    = (bitmap[0] & (1 << 2)) != 0;  // Sensor ID 0x03
    sensors.gyroscope                      = (bitmap[0] & (1 << 3)) != 0;  // Sensor ID 0x04
    sensors.barometer                      = (bitmap[0] & (1 << 5)) != 0;  // Sensor ID 0x06
    sensors.gravity                        = (bitmap[1] & (1 << 1)) != 0;  // Sensor ID 0x09
    sensors.linear_acceleration            = (bitmap[1] & (1 << 2)) != 0;  // Sensor ID 0x0A
    sensors.rotation_vector                = (bitmap[1] & (1 << 3)) != 0;  // Sensor ID 0x0B

    sensors.uncalibrated_magnetometer      = (bitmap[1] & (1 << 6)) != 0;  // Sensor ID 0x0E
    sensors.game_rotation_vector           = (bitmap[1] & (1 << 7)) != 0;  // Sensor ID 0x0F
    sensors.uncalibrated_gyroscope         = (bitmap[2] & (1 << 0)) != 0;  // Sensor ID 0x10
    sensors.geomagnetic_rotation_vector    = (bitmap[2] & (1 << 4)) != 0;  // Sensor ID 0x14
    sensors.tilt_detector                  = (bitmap[2] & (1 << 6)) != 0;  // Sensor ID 0x16

    #ifdef DEBUG
    SerialUSB.println("Queried Physical Sensors Present:");
	SerialUSB.print("Bitmap: "); SerialUSB.print(bitmap[0], HEX); SerialUSB.print(" "); SerialUSB.println(bitmap[1], HEX);
    SerialUSB.print("Accelerometer: "); SerialUSB.println(sensors.accelerometer);
    SerialUSB.print("Magnetometer: "); SerialUSB.println(sensors.magnetometer);
    SerialUSB.print("Orientation: "); SerialUSB.println(sensors.orientation);
    SerialUSB.print("Gyroscope: "); SerialUSB.println(sensors.gyroscope);
    SerialUSB.print("Barometer: "); SerialUSB.println(sensors.barometer);
    SerialUSB.print("Gravity: "); SerialUSB.println(sensors.gravity);
    SerialUSB.print("Linear Acceleration: "); SerialUSB.println(sensors.linear_acceleration);
    SerialUSB.print("Rotation Vector: "); SerialUSB.println(sensors.rotation_vector);
    SerialUSB.print("Uncalibrated Magnetometer: "); SerialUSB.println(sensors.uncalibrated_magnetometer);
    SerialUSB.print("Game Rotation Vector: "); SerialUSB.println(sensors.game_rotation_vector);
    SerialUSB.print("Uncalibrated Gyroscope: "); SerialUSB.println(sensors.uncalibrated_gyroscope);
    SerialUSB.print("Geomagnetic Rotation Vector: "); SerialUSB.println(sensors.geomagnetic_rotation_vector);
    SerialUSB.print("Tilt Detector: "); SerialUSB.println(sensors.tilt_detector);
    #endif

    return sensors;
}
*/