/**
 * @file Accelerometer_Example.ino
 *
 * Example sketch for the Naviguider I2C Compass module using the 
 * Naviguider_Compass_I2C library.
 *
 * This example demonstrates how to initialize the Naviguider compass module,
 * and read accelerometer data over I2C. The example is designed to run on 
 * the SAMD21-Mini and uses `SerialUSB` for serial communication. If you are 
 * using a standard Arduino board, replace all instances of `SerialUSB` 
 * with `Serial`.
 *
 * Marine Applied Research & Exploration (MARE) develops and shares this 
 * code to support the exploration and documentation of deep-water 
 * ecosystems, contributing to their conservation and management. To 
 * sustain our mission and initiatives, please consider donating at 
 * https://maregroup.org/donate.
 *
 * Created by Isaac Assegai for Marine Applied Research & Exploration.
 *
 * This example is distributed under the BSD license. Redistribution must
 * retain this notice and accompanying license text.
 */


// Include the library for the Naviguider Compass
#include <Naviguider_Compass_I2C.h> 

// Create an instance of the NaviguiderCompass class
NaviguiderCompass compass; 

// Initialize the serial port for debugging
void setup() {

    // On the SAMD21-Mini, use SerialUSB. For standard Arduino boards, use Serial.
    SerialUSB.begin(115200);

    // Wait for the serial connection to be established
    while (!SerialUSB); 

    // Initialize the Naviguider Compass module
    if (!compass.begin()) {

        // Print an error message and stop execution if initialization fails
        SerialUSB.println("Failed to initialize Naviguider Compass!");

        // Halt the program
        while (1); 
    }

    // If initialization is successful, print a confirmation message
    SerialUSB.println("Naviguider Compass initialized.");
}


void loop() {

    // Variables to store accelerometer data
    int16_t x, y, z;      // X, Y, Z axes values
    uint8_t accuracy;     // Accuracy level of the reading

    // Read accelerometer data from the Naviguider Compass
    if (compass.readAccelerometer(x, y, z, accuracy)) {

        // Print accelerometer data to the serial monitor
        SerialUSB.print("Accel X: "); SerialUSB.print(x);
        SerialUSB.print(", Y: "); SerialUSB.print(y);
        SerialUSB.print(", Z: "); SerialUSB.print(z);
        SerialUSB.print(", Accuracy: "); SerialUSB.println(accuracy);
    } else {
      
        // Print an error message if reading fails
        SerialUSB.println("Failed to read accelerometer data.");
    }

    // Delay for 500 milliseconds before reading again
    delay(500);
}