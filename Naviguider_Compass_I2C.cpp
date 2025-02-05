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
 
 /**
  * @brief Static flag indicating whether an interrupt has occurred.
  *
  * This volatile boolean flag is set to true inside the interrupt service routine (ISR)
  * when an external interrupt is triggered. It is checked within the main program loop
  * to determine if sensor data needs to be processed. Since it is modified within an ISR,
  * it is declared as `volatile` to prevent compiler optimizations that could ignore changes
  * made outside the main execution flow.
  */
volatile bool NaviguiderCompass::interruptFlag = false;


/**
 * @brief Static structure storing the status of physical sensors.
 *
 * This structure holds real-time status information about physical sensors,
 * such as their sample rates, dynamic ranges, and operational states. It is
 * used throughout the program to access sensor-specific details and ensure
 * correct operation.
 */
PhysicalSensorStatusStruct NaviguiderCompass::PhysicalSensorStatus;


/**
 * @brief Static instance pointer to the NaviguiderCompass class.
 *
 * This static pointer allows access to the single instance of the
 * NaviguiderCompass class from within static methods, such as the interrupt
 * handler. It enables a proper object-oriented approach when dealing with
 * interrupts that require access to class member functions.
 */
NaviguiderCompass* NaviguiderCompass::instance = nullptr;


/**
 * @brief Constructs a NaviguiderCompass object with a specified interrupt pin.
 *
 * This constructor initializes the NaviguiderCompass instance and assigns the
 * provided host interrupt pin. It also configures the interrupt pin as an input.
 *
 * @param hostInterruptPin The digital pin number used for receiving sensor interrupts.
 */
NaviguiderCompass::NaviguiderCompass(int hostInterruptPin)
    : _hostInterruptPin(hostInterruptPin) {

    // Assign instance pointer to this object
    instance = this;  

    // Set the host interrupt pin as input
    pinMode(_hostInterruptPin, INPUT);
}


/**
 * @brief Initializes the Naviguider Compass device.
 *
 * This function sets up the I2C communication, configures the interrupt pin,
 * starts the CPU, and initializes sensor settings. It also prints debug
 * information if enabled.
 *
 * @param wire Pointer to the TwoWire (I2C) instance to use for communication.
 * @param address The I2C address of the Naviguider device.
 * @return True if initialization was successful, false otherwise.
 */
bool NaviguiderCompass::begin(TwoWire *wire, uint8_t address) {

    // Construct the I2C device
    m_i2c_dev = new Adafruit_I2CDevice(address, wire);

    // Start I2C communication
    if (!m_i2c_dev->begin()) {
        return false; // Failed to initialize I2C device
    }
	
    // Set base returnVal
	bool returnVal = true;
	
	// Get Naviguider Fusion Co-processor Device ID
	fusionCoprocessor = getFusionCoprocessor();
	
    // Setup the interrupt pin, and interrupt service routine
    pinMode(_hostInterruptPin, INPUT); // Set D2 as input for interrupt
    attachInterrupt(digitalPinToInterrupt(_hostInterruptPin), NaviguiderCompass::interruptHandler, RISING); // Attach interrupt to pin 2

     // Start the CPU
    startCPU();

	// Print Sensor Information to Serial Console, if DEBUG is defined.
	#ifdef DEBUG
	printNaviguiderStatusRegister();
	printNaviguiderSensorStatus();
	printPhysicalSensorStatus();
	printPhysicalSensorInformation();
    printSensorInformation();
    printSensorConfiguration();
	#endif
    
    // Set the rate for the first 4 sensors
    setSensorRate( 1, 0x0A); //0a
    setSensorRate(2, 0x64); //64
    setSensorRate(3, 0x0F); //0f
    setSensorRate(4, 0x0F); //0f
	
    // Let the caller know we have succeeded in initializing the Naviguider
    return returnVal; 
}


/**
 * @brief Retrieves the compass heading in degrees.
 *
 * The heading is computed from the sensor's orientation data and is wrapped
 * within the range of [0, 360] degrees.
 *
 * @return The heading in degrees.
 */
float NaviguiderCompass::getHeading() {
    return fmod(orientationData.x + 360.0f, 360.0f);
}


/**
 * @brief Retrieves the pitch angle of the sensor.
 *
 * The pitch represents the sensor's tilt forward or backward.
 *
 * @return The pitch angle in degrees.
 */
float NaviguiderCompass::getPitch() {
    return orientationData.y;// *0.010986;
}


/**
 * @brief Retrieves the roll angle of the sensor.
 *
 * The roll represents the sensor's tilt left or right.
 *
 * @return The roll angle in degrees.
 */
float NaviguiderCompass::getRoll() {
    return orientationData.z;// *0.010986;
}


/**
 * @brief Retrieves the accuracy of the heading measurement.
 *
 * This value represents the estimated accuracy of the heading
 * calculation based on sensor fusion confidence.
 *
 * @return The heading accuracy in degrees.
 */
float NaviguiderCompass::getHeadingAccuracy() {
    return orientationData.extraInfo;
}


/**
 * @brief Retrieves the yaw rate (rate of rotation around the vertical axis).
 *
 * The yaw rate is converted from the sensor's raw gyroscope data
 * from radians per second to degrees per second.
 *
 * @return The yaw rate in degrees per second.
 */
float NaviguiderCompass::getYawRate() {
    float radiansPerSecond = gyroData.z;// *0.0010647; // per datasheet

    // Convert radians to degrees (180/pi = 57.2958f)
    float degreesPerSecond = radiansPerSecond * 57.2958f;

    return degreesPerSecond;
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
        #ifdef DEBUG_LEVEL_3
        SerialUSB.println("Failed to read Host Status registers");
        #endif
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
        #ifdef DEBUG_LEVEL_3
        SerialUSB.println("Failed to read Error Register");
        #endif
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
        #ifdef DEBUG_LEVEL_3
        SerialUSB.println("Failed to read Bytes Remaining register");
        #endif
        return;
    }

    #ifdef DEBUG_LEVEL_3
    SerialUSB.print("Bytes Remaining:   ");
    SerialUSB.print(bytesAvailable);
    SerialUSB.print(", ");

    #endif
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
        #ifdef DEBUG_LEVEL_3
        SerialUSB.println("Failed to read sensor status (Bank 1)");
        #endif
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
        #ifdef DEBUG_LEVEL_3
        SerialUSB.println("Failed to read physical sensor status.");
        #endif
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
        #ifdef DEBUG_LEVEL_3
        SerialUSB.println("Failed to read physical sensor presence bitmap.");
        #endif
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
           
            // Setup a buffer for reading into
            uint8_t buffer[16];

            // Read the parameter
            if (!readParameter(
                NAVIGUIDER_PARAMETER_PAGE_SYSTEM,
                &paramPresence,
                1,
                buffer))
            {
                #ifdef DEBUG_LEVEL_3
                SerialUSB.print("Failed to read information for sensor ID: ");
                SerialUSB.println(sensorId);
                #endif
                continue;
            }

            // Copy from buffer to sensorInfo
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


/**
* @brief Retrieves the name of a sensor based on its sensor ID.
*
* This function returns a human-readable string for the given sensor ID.
* If the ID is out of range, it returns "unknown sensor".
*
* @param sensorId The ID of the sensor.
* @return A pointer to a constant string representing the sensor's name.
*/
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
        case NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED: return "magnetic field uncalibrated";
        case NAVIGUIDER_SENSOR_TYPE_GAME_ROTATION_VECTOR:        return "game rotation vector";
        case NAVIGUIDER_SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:      return "gyroscope uncalibrated";
        case NAVIGUIDER_SENSOR_TYPE_SIGNIFICANT_MOTION:          return "significant motion";
        case NAVIGUIDER_SENSOR_TYPE_STEP_DETECTOR:               return "step detector";
        case NAVIGUIDER_SENSOR_TYPE_STEP_COUNTER:                return "step counter";
        case NAVIGUIDER_SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR: return "geomagnetic rotation vector";
        case NAVIGUIDER_SENSOR_TYPE_HEART_RATE:                  return "heart rate -OR- car detector";
        case NAVIGUIDER_SENSOR_TYPE_TILT_DETECTOR:               return "tilt detector";
        case NAVIGUIDER_SENSOR_TYPE_WAKE_GESTURE:                return "wake gesture";
        case NAVIGUIDER_SENSOR_TYPE_GLANCE_GESTURE:              return "glance gesture";
        case NAVIGUIDER_SENSOR_TYPE_PICK_UP_GESTURE:             return "pick up gesture";
        case NAVIGUIDER_SENSOR_TYPE_RAW_ACCEL:                   return "raw accel";
        case NAVIGUIDER_SENSOR_TYPE_RAW_MAG:                     return "raw mag";
        case NAVIGUIDER_SENSOR_TYPE_RAW_GYRO:                    return "raw gyro";
        case NAVIGUIDER_SENSOR_TYPE_ACTIVITY:                    return "activity";
        case NAVIGUIDER_SENSOR_TYPE_CAR_MAG_DATA:                return "car detect mag data (uT)";

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

    #ifdef DEBUG_LEVEL_3
    SerialUSB.println("Reading Sensor Information");
    #endif

    // Setup a buffer to read into
    uint8_t sensorInfoBuffer[numSensors * 16];
    
    // Read the parameter
    if (!readParameter(
        NAVIGUIDER_PARAMETER_PAGE_SENSOR_INFO,
        paramPresence, // Parameter list
        numSensors, // Number of parameters
        sensorInfoBuffer))
    {
        #ifdef DEBUG_LEVEL_3
        SerialUSB.println("Error: Failed to read sensor information.");
        #endif
        return;
    }

    #ifdef DEBUG_LEVEL_3
    for (int j = 0; j < sizeof(sensorInfoBuffer) - 1; j++) {
        SerialUSB.print("sensorInfoBuffer: "); SerialUSB.println(sensorInfoBuffer[j], HEX);

    }

    SerialUSB.println("DONE READING Reading Sensor Information");
    #endif

    // Copy the data from the buffer into the sensorInformation structure
    memcpy(sensorInformation, sensorInfoBuffer, sizeof(sensorInfoBuffer));

    // Mark that we now have valid sensor information
    haveSensorInformation = true;

    // Extract maximum data rates for individual sensors
    magMaxRate = sensorInformation[NAVIGUIDER_SENSOR_TYPE_MAGNETIC_FIELD].maxRate;
    accelMaxRate = sensorInformation[NAVIGUIDER_SENSOR_TYPE_ACCELEROMETER].maxRate;
    gyroMaxRate = sensorInformation[NAVIGUIDER_SENSOR_TYPE_GYROSCOPE].maxRate;
}


/**
 * @brief Prints out all naviguider sensor information.
 */
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
    // Print table footer
    SerialUSB.println("+-----+----------------------------------+---------+-------+-------+-----+----------+------+");
}


/**
 * @brief Prints the sensor configuration details in a formatted table.
 *
 * This function retrieves and displays the configuration settings of all available sensors.
 * It ensures that sensor information is available before proceeding and then queries the
 * sensor configuration parameters. The information is printed in a well-aligned table format.
 *
 * The displayed parameters include:
 * - Sensor name
 * - Sample rate (Hz)
 * - Maximum report latency
 * - Sensitivity threshold
 * - Dynamic range
 */
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

    char buffer[100]; // Buffer to store formatted output

    // Iterate over available sensor information
    for (uint8_t i = 0; i < (sizeof(sensorInformation) / sizeof(sensorInformation[0])); i++)
    {
        if (sensorInformation[i].sensorId > 0)
        {
            // Format and print the row correctly aligned
            sprintf(buffer, "| %-32s | %5u | %5u | %11u | %6u |",
                getSensorName(sensorInformation[i].sensorId),
                sensorConfiguration[i].sampleRate,
                sensorConfiguration[i].maxReportLatency,
                sensorConfiguration[i].changeSensitivity,
                sensorConfiguration[i].dynamicRange);
            SerialUSB.println(buffer);
        }
    }

    // Print table footer
    SerialUSB.println("+----------------------------------+-------+-------+-------------+--------+");
}


/**
 * @brief Reads sensor data from the Naviguider and processes pending interrupts.
 *
 * This function checks if an interrupt has been triggered and, if so, processes
 * the available sensor data from the FIFO buffer. It then re-enables the interrupt
 * to continue capturing new sensor events.
 *
 * - If an interrupt has occurred, it clears the interrupt flag.
 * - Reads available data from the FIFO buffer.
 * - Parses the retrieved FIFO data.
 * - Reattaches the interrupt handler to resume normal operation.
 */
void NaviguiderCompass::readSensors() {
	
    // Check if we need to service an interrupt that happened since we last readSensors
    if (interruptFlag) {

        // Set the interrupt flag to false now that we've started processed the event and sensor data
        interruptFlag = false;

        // Read Fifo
        uint32_t bytesRead = readFifo();

        if (bytesRead > 0) {
            // Parse Fifo
            parseFifo(bytesRead);
        }
       
        // Enable Interrupts
        attachInterrupt(digitalPinToInterrupt(_hostInterruptPin), NaviguiderCompass::interruptHandler, RISING); // Attach interrupt to pin 2

    }
	
    // return to user
    return;
}


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
uint32_t NaviguiderCompass::readFifo() {
    uint16_t bytesAvailable = 0;

    // Read the number of bytes available in the FIFO
    uint8_t reg = NAVIGUIDER_REG_BYTES_REMAINING_LSB; // FIFO bytes remaining register
    if (!m_i2c_dev->write_then_read(&reg, 1, (uint8_t *)&bytesAvailable, 2)) {
        #ifdef DEBUG_LEVEL_2
        SerialUSB.println("Error reading remaining bytes left, for FIFO!");
        #endif
        return 0; // Communication error
    }

    // Ensure bytesAvailable is within a reasonable range
    if (bytesAvailable == 0 || bytesAvailable > sizeof(fifoBuffer)) {
        #ifdef DEBUG_LEVEL_2
        SerialUSB.println("0 Bytes available read from bytes available register!");
        #endif
        return 0; // No data or invalid size
    }

    // Now read the FIFO data from the appropriate register
    uint8_t fifoDataReg = 0;//NAVIGUIDER_REG_EVENT_STATUS; // Replace with the actual FIFO data register

    if (!m_i2c_dev->write_then_read(&fifoDataReg, 1, fifoBuffer, bytesAvailable)) {
        #ifdef DEBUG_LEVEL_2
        SerialUSB.println("Error reading fifo register!");
        #endif
        return 0; // Read error
    }

    return bytesAvailable; // Return the total bytes read
}


/**
 * @brief Parses the FIFO buffer to process sensor data.
 *
 * This function iterates through the FIFO buffer, extracting and processing
 * sensor data in discrete blocks. It calls `parseNextFifoBlock()` to handle
 * individual blocks of data while tracking how many bytes have been used.
 *
 * @param size The total number of bytes available in the FIFO buffer.
 * @return The number of bytes successfully processed from the FIFO buffer.
 */
uint32_t NaviguiderCompass::parseFifo(uint32_t size) {
    uint32_t index = 0;
    uint32_t bytesUsed;
    uint32_t bytesRemaining = size;

    if (size == 0) return size;

    // parse each block in the fifo until there is no data remaining
    do {
        bytesUsed = parseNextFifoBlock(&fifoBuffer[index], bytesRemaining);
        index += bytesUsed;
        bytesRemaining -= bytesUsed;
    } while (bytesUsed > 0 && bytesRemaining > 0);

    // Let caller know how many bytes we processed
    return size - bytesRemaining;
}


/**
 * @brief Parses a single block of data from the FIFO buffer.
 *
 * This function processes sensor data by identifying the sensor type and extracting
 * the corresponding data. It updates relevant sensor structures based on the sensor ID
 * found in the first byte of the buffer.
 *
 * - Handles timestamps and timestamp overflows.
 * - Processes meta events for debugging and error handling.
 * - Extracts data for various sensors including magnetometer, accelerometer, orientation, gyroscope, and rotation vector.
 * - Uses appropriate scaling factors to convert raw sensor values to meaningful units.
 *
 * @param buffer Pointer to the FIFO buffer containing the data block.
 * @param size The size of the remaining FIFO buffer to be processed.
 * @return The number of bytes consumed from the buffer while processing the block.
 */
uint32_t NaviguiderCompass::parseNextFifoBlock(uint8_t* buffer, uint32_t size) {

    // Get the sensor id from the first byte in the buffer
    uint8_t sensorId = buffer[0];

    switch (sensorId) {
        
        // Check if timestamp overflow has a value
        case NAVIGUIDER_SENSOR_TYPE_TIMESTAMP_OVERFLOW:
        case NAVIGUIDER_SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
        {
            uint16_t uPacket;
            memcpy(&uPacket, &buffer[1], sizeof(uint16_t)); // Safe read

            timestampTemp[1] = uPacket;
            timestampTemp[0] = 0;

            timestamp = ((uint32_t)timestampTemp[1] << 16) | timestampTemp[0];
            return 3;
        }

        // Check if timestamp has a value
        case NAVIGUIDER_SENSOR_TYPE_TIMESTAMP:
        case NAVIGUIDER_SENSOR_TYPE_TIMESTAMP_WAKE:
        {
            uint16_t uPacket;
            memcpy(&uPacket, &buffer[1], sizeof(uint16_t)); // Safe read

            timestampTemp[0] = uPacket;
            timestamp = *(uint32_t*)timestampTemp;
            return 3;
        }

        // Check For Meta Events (this is where we can look for an error, and process it)
        case NAVIGUIDER_REG_META_EVENT:
        case NAVIGUIDER_REG_META_EVENT_WAKE:
        {
            SerialUSB.println("NAVIGUIDER_REG_META_EVENT/WAKE");
            SerialUSB.print(buffer[1], HEX); SerialUSB.print("    Meta Event Name: "); SerialUSB.println(getMetaEventName(buffer[1]));
            SerialUSB.print("    Sensor Name "); SerialUSB.println(getSensorName(buffer[2]));
            SerialUSB.print("    buffer[3]: "); SerialUSB.println(buffer[3], HEX);
            //printf("%u NAVIGUIDER_REG_META_EVENT/WAKE %s:, %s/%u, %u\n\r", timestamp, em7186_meta_event_name[buffer[1]], em7186_sensor_name[buffer[2]], buffer[2], buffer[3]);
            return 4;
        }

        // Get Manetometer data
        case NAVIGUIDER_REG_MAGNETOMETER:
        case NAVIGUIDER_REG_MAGNETOMETER_WAKE:
        {
            get3AxisSensorData(&magData, 1000 / powf(2.0f, 15.0f), buffer);
            #ifdef DEBUG_LEVEL_3
            SerialUSB.print("Magnetometer Data - X: "); SerialUSB.print(magData.x, HEX); SerialUSB.print("  Y: "); SerialUSB.print(magData.y, HEX); SerialUSB.print("  Z: "); SerialUSB.print(magData.z, HEX); SerialUSB.print("  extra: "); SerialUSB.println(magData.extraInfo, HEX);
            #endif

            return 8;
        }

        // Get Accelerometer data
        case NAVIGUIDER_REG_ACCELEROMETER:
        case NAVIGUIDER_REG_ACCELEROMETER_WAKE:
        {
            get3AxisSensorData(&accelData, 9.81f * 4.0f / powf(2.0f, 15.0f), buffer);
            #ifdef DEBUG_LEVEL_3
            SerialUSB.print("Accel Data - X: "); SerialUSB.print(accelData.x, HEX); SerialUSB.print("  Y: "); SerialUSB.print(accelData.y, HEX); SerialUSB.print("  Z: "); SerialUSB.print(accelData.z, HEX); SerialUSB.print("  extra: "); SerialUSB.println(accelData.extraInfo, HEX);
            #endif

            return 8;
        }

        // Get orientation Data
        case NAVIGUIDER_REG_ORIENTATION:
        case NAVIGUIDER_REG_ORIENTATION_WAKE:
        {
            get3AxisSensorData(&orientationData, 360.0f / powf(2.0f, 15.0f), buffer);
            #ifdef DEBUG_LEVEL_3
            SerialUSB.print("Orientation - X: "); SerialUSB.print(orientationData.x, HEX); SerialUSB.print("  Y: "); SerialUSB.print(orientationData.y, HEX); SerialUSB.print("  Z: "); SerialUSB.print(orientationData.z, HEX); SerialUSB.print("  extra: "); SerialUSB.println(orientationData.extraInfo, HEX);
            #endif

            return 8;
        }

        // Get gyro data
        case NAVIGUIDER_REG_GYROSCOPE:
        case NAVIGUIDER_REG_GYROSCOPE_WAKE:
        {
            get3AxisSensorData(&gyroData, (3.1415927f / 180.0f) * 2000.0f / powf(2.0f, 15.0f), buffer);
            #ifdef DEBUG_LEVEL_3
            SerialUSB.print("gyroData - X: "); SerialUSB.print(gyroData.x, HEX); SerialUSB.print("  Y: "); SerialUSB.print(gyroData.y, HEX); SerialUSB.print("  Z: "); SerialUSB.print(gyroData.z, HEX); SerialUSB.print("  extra: "); SerialUSB.println(gyroData.extraInfo, HEX);
            #endif

            return 8;
        }

        // Get rotation Vector Data
        case NAVIGUIDER_SENSOR_TYPE_ROTATION_VECTOR:
        case NAVIGUIDER_SENSOR_TYPE_ROTATION_VECTOR_WAKE:
        {
            SensorData4Axis rotationVector;
            getRotationVector(&rotationVector, 1.0f / powf(2.0f, 14.0f), buffer);
            return 11;
        }

        // Check for 0, or defaults
        case 0:
            return size;
        default:
            return size;
    }
}


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
uint8_t NaviguiderCompass::getRotationVector(SensorData4Axis* rv, float quaternionScale, uint8_t* buffer)
{
    // Temporary struct to hold raw data from buffer
    RotationVectorRaw rawData;

    // Copy raw sensor data from buffer to the struct
    memcpy(&rawData, &buffer[1], sizeof(rawData));

    // Apply scaling to convert raw quaternion values into meaningful data
    rv->x = rawData.x * quaternionScale;
    rv->y = rawData.y * quaternionScale;
    rv->z = rawData.z * quaternionScale;
    rv->w = rawData.w * quaternionScale;

    // Convert accuracy value to radians using scale factor (π / 2^14)
    rv->extraInfo = rawData.accuracy * (M_PI / powf(2.0f, 14.0f));

    return 1; // Indicate success
}


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
uint8_t NaviguiderCompass::get3AxisSensorData(SensorData3Axis* data, float scale, uint8_t* buffer)
{
    // Temporary structure to hold raw sensor data
    SensorData3Axis_RAW rawData;

    // Copy raw data from the buffer, starting from index 1
    memcpy(&rawData, &buffer[1], sizeof(rawData));

    // Apply scaling factor to convert raw integer values into floating-point values
    data->x = (float)rawData.x * scale;
    data->y = (float)rawData.y * scale;
    data->z = (float)rawData.z * scale;

    // Store additional sensor status information
    data->extraInfo = rawData.status;

    return 1; // Indicate success
}


/**
 * @brief Retrieves the name of a meta event based on its event ID.
 *
 * This function returns a human-readable string for the given meta event ID.
 * If the ID is out of the defined range, it returns "Unknown Meta Event".
 *
 * @param eventId The ID of the meta event.
 * @return A pointer to a constant string representing the meta event's name.
 */
const char* NaviguiderCompass::getMetaEventName(uint8_t eventId) {
    switch (eventId) {
    case 0:  return "META EVENT NOOP";
    case 1:  return "META_EVENT_FLUSH_COMPLETE";
    case 2:  return "META_EVENT_SAMPLE_RATE_CHANGED";
    case 3:  return "META_EVENT_POWER_MODE_CHANGED";
    case 4:  return "META_EVENT_ERROR";
    case 5:  return "META_EVENT_ALGORITHM_EVENT";
    case 6:  return "META_EVENT_CAL_STATUS_CHANGED";
    case 11: return "META_EVENT_SENSOR_EVENT";
    case 12: return "META_EVENT_FIFO_OVERFLOW";
    case 13: return "META_EVENT_DYNAMIC_RANGE_CHANGED";
    case 14: return "META_EVENT_FIFO_WATERMARK";
    case 15: return "META_EVENT_SELF_TEST_RESULT";
    case 16: return "META_EVENT_INITIALIZED";
    case 17: return "META_EVENT_TRANSFER_CAUSE";
    default: return "Unknown Meta Event"; // Covers undefined events (7-10 and beyond)
    }
}


/**
 * @brief Static interrupt handler for the Naviguider Compass.
 *
 * This function is called when an interrupt is triggered on the host interrupt pin.
 * Since static methods cannot access instance variables directly, this function
 * calls the instance-specific handler via the static instance pointer.
 */
void NaviguiderCompass::interruptHandler() {
    // Check if an instance of NaviguiderCompass has been created
    if (instance) {
        instance->handleInterrupt();  // Call the instance-specific interrupt handler
    }
}


/**
 * @brief Instance-specific interrupt handler.
 *
 * This function is responsible for handling the interrupt triggered by the host interrupt pin.
 * It sets the interrupt flag and disables further interrupts until they are re-enabled
 * to prevent multiple triggers before the event is processed.
 */
void NaviguiderCompass::handleInterrupt() {
    // Set the interrupt flag to indicate an interrupt event has occurred
    interruptFlag = true;

    // Detach the interrupt to prevent multiple triggers before processing
    detachInterrupt(digitalPinToInterrupt(_hostInterruptPin));
}


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
uint32_t NaviguiderCompass::setSensorRate(uint8_t sensorId, uint8_t rate)
{

    ////SerialUSB.print("setSensorRate: "); SerialUSB.print(sensorId, HEX); SerialUSB.print(" : "); SerialUSB.println(rate);
    uint8_t paramPage;
    ParameterInformation param;
    paramPage = NAVIGUIDER_PARAMETER_PAGE_SENSOR_CONFIG;

    param.ParameterNumber = sensorId;
    param.DataSize = 2; // Set the size of the parameter

    // Write the parameter to the sensor configuration
    uint32_t isWritten = 0;

    isWritten = writeParameter(paramPage, &param, 1, &rate);

    return isWritten;
}


/**
 * @brief Writes one or more parameters to the Naviguider sensor.
 *
 * This function updates the specified sensor parameters by selecting the appropriate
 * parameter page, writing values to the parameter load register, and committing the
 * changes through the parameter request register. It then waits for acknowledgment
 * from the sensor before proceeding.
 *
 * The procedure follows these steps:
 * 1. Selects the appropriate parameter page.
 * 2. Writes the parameter values to the load register.
 * 3. Sends a parameter request to apply the changes.
 * 4. Polls the parameter acknowledgment register to confirm the write operation.
 * 5. Resets the request and page select registers to complete the operation.
 *
 * If any step fails, the function returns 0 to indicate failure. A return value of
 * 1 indicates a successful write.
 *
 * @param page The parameter page number to modify.
 * @param paramList Pointer to an array of ParameterInformation structures specifying the parameters to write.
 * @param numParams The number of parameters to write.
 * @param values Pointer to the buffer containing the parameter values to be written.
 * @return uint32_t 1 if successful, 0 if an error occurred.
 */
uint32_t NaviguiderCompass::writeParameter(uint8_t page, const ParameterInformation* paramList, uint8_t numParams, uint8_t* values) {

    uint8_t i, paramAck, paramNum, pageSelectValue;
    uint16_t valIndex = 0;


   
    // Loop through all params
    for (i = 0; i < numParams; i++) {


        // Choose which page to select
        pageSelectValue = page | (paramList[i].DataSize << 4);

        // Step 1: Write the Page Number to Parameter Page Select Register
        uint8_t reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
        if (!m_i2c_dev->write(&pageSelectValue, 1, true, &reg, 1))
        {
            return 0;
        }

        // Step2: Write the Parameter Load register with the values
        reg = NAVIGUIDER_REG_LOAD_PARAM_BYTE_0;
        if (!m_i2c_dev->write(&values[valIndex], paramList[i].DataSize, true, &reg, 1))
        {
            return 0;
        }

        // Step 3: Write to Param request Register
        paramNum = paramList[i].ParameterNumber | 0x80;
        reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
        if (!m_i2c_dev->write(&paramNum, 1, true, &reg, 1))
        {
            return 0;
        }

        // Step 4, Read from the param ack register:
         // Poll Parameter Acknowledge Register (Use do-while like the original)
        uint8_t ack = 0;
        reg = NAVIGUIDER_REG_PARAMETER_ACKNOWLEDGE;
        do {
            if (!m_i2c_dev->write_then_read(&reg, 1, &ack, 1))
            {
               
                return 0;
            }

            if (ack == 0x80)
            {
                // Reset registers
                uint8_t resetVal = 0;
                reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
                m_i2c_dev->write(&resetVal, 1, true, &reg, 1);

                reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
                m_i2c_dev->write(&resetVal, 1, true, &reg, 1);

                return 0;
            }
        } while (ack != paramNum);

        valIndex += paramList[i].DataSize;
    }

    // Step 5: End the parameter transfer procedure by writing 0 to the Parameter Request register
    uint8_t resetVal = 0;
    uint8_t reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
    if (!m_i2c_dev->write(&resetVal, 1, true, &reg, 1))
    {
        return 0;
    }


    // Step 6: End the parameter transfer procedure by writing 0 to the Parameter Request register
    resetVal = 0;
    reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
    if (!m_i2c_dev->write(&resetVal, 1, true, &reg, 1))
    {
        return 0;
    }

    return 1;
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

    #ifdef DEBUG_LEVEL_3
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

    #endif


    uint8_t pageSelectValue, paramAck;
    uint16_t valueIndex = 0;

    
    for (uint8_t i = 0; i < numParams; i++)
    {
		// Step 1: Write the Page Number to Parameter Page Select Register
		pageSelectValue = page;
		uint8_t reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;

		if (!m_i2c_dev->write(&pageSelectValue, 1, true, &reg, 1))
		{
			return false;
		}

        #ifdef DEBUG_LEVEL_3
		SerialUSB.print("readParameter: Successfully set parameter page (0x54) to: 0x");
		SerialUSB.println(pageSelectValue, HEX);
        #endif

        // Step 2: Write the Parameter Number to Parameter Request Register
        reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
		
        #ifdef DEBUG_LEVEL_3
		SerialUSB.print("writing to register (0x64) a value of: 0x");
		SerialUSB.println(paramList[i].ParameterNumber, HEX);
        #endif
		
        if (!m_i2c_dev->write(&paramList[i].ParameterNumber, 1, false, &reg, 1))
        {
            #ifdef DEBUG_LEVEL_3
            SerialUSB.println("readParameter: Failed to send Parameter Request");
            #endif
            return false;
        }

        // Poll Parameter Acknowledge Register (Use do-while like the original)
		uint8_t ack = 0;
		reg = NAVIGUIDER_REG_PARAMETER_ACKNOWLEDGE;
		do {
			if (!m_i2c_dev->write_then_read(&reg, 1, &ack, 1))
			{
                #ifdef DEBUG_LEVEL_3
				SerialUSB.println("I2C read failed while polling ACK (0x3A)");
                #endif
				return false;
			}

			if (ack == 0x80)
			{
                #ifdef DEBUG_LEVEL_3
				SerialUSB.println("Error: Parameter request not supported (ACK = 0x80)");
                #endif

				// Reset registers
				uint8_t resetVal = 0;
				reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
				m_i2c_dev->write(&resetVal, 1, true, &reg, 1);
				
				reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
				m_i2c_dev->write(&resetVal, 1, true, &reg, 1);

				return false;
			}
		} while (ack != paramList[i].ParameterNumber);

        #ifdef DEBUG_LEVEL_3
        SerialUSB.print("Parameter Acknowledged: 0x");
        SerialUSB.println(ack, HEX);

        SerialUSB.print("Reading Register NAVIGUIDER_REG_SAVED_PARAM_BYTE_0 (0x3B), into values for return, NUMBYTES :");
        SerialUSB.println(paramList[i].DataSize);
        #endif


        // Step 4: Read the Parameter Data from Saved Parameter Bytes, i think there's an issue here reading data
        reg = NAVIGUIDER_REG_SAVED_PARAM_BYTE_0;
        if (!m_i2c_dev->write_then_read(&reg, 1, &values[valueIndex], paramList[i].DataSize))
        {
            #ifdef DEBUG_LEVEL_3
            SerialUSB.println("readParameter: Failed to read Parameter Data");
            #endif
            return false;
        }

        valueIndex += paramList[i].DataSize;
    }

    // Step 6: End the parameter transfer procedure by writing 0 to the Parameter Request register
    uint8_t resetVal = 0;
    uint8_t reg = NAVIGUIDER_REG_PARAMETER_REQUEST;
    if (!m_i2c_dev->write(&resetVal, 1, true, &reg, 1))
    {
        return false;
    }
	
	// Step 7: End the parameter transfer procedure by writing 0 to the Parameter Request register
    resetVal = 0;
    reg = NAVIGUIDER_REG_PARAMETER_PAGE_SELECT;
    if (!m_i2c_dev->write(&resetVal, 1, true, &reg, 1))
    {
        return false;
    }

    // We've completed the read, return true
    return true;
}


/**
 * @brief Queries the fusion co-processor type and returns its name.
 *
 * This function reads the product ID register of the Naviguider sensor to determine
 * the type of fusion co-processor present. It then interprets the retrieved ID
 * and returns a corresponding string representation of the co-processor type.
 * If communication fails or an unknown ID is found, it returns "NO CO-PROCESSOR FOUND".
 *
 * @return A string representing the detected fusion co-processor type.
 */
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


/**
 * @brief Starts the CPU of the Naviguider sensor.
 *
 * This function writes to the CHIP_CONTROL register to initiate the CPU start sequence.
 * It ensures that the CPU is properly started by sending a specific command via I2C.
 *
 * - Writes `0x01` to the NAVIGUIDER_REG_CHIP_CONTROL register to start the CPU.
 * - Checks if the I2C write operation was successful.
 * - Returns `true` if the CPU was successfully started, otherwise returns `false`.
 * - If DEBUG mode is enabled, prints messages indicating success or failure.
 *
 * @return `true` if the CPU started successfully, `false` otherwise.
 */
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