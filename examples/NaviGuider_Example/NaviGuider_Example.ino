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
 * Created by Isaac Assegai Travers for Marine Applied Research & Exploration.
 *
 * This example is distributed under the BSD license. Redistribution must
 * retain this notice and accompanying license text.
 */


#include "Naviguider_Compass_I2C.h"

// Define which pin you will hook to the naviguider interrupt
#define INTERRUPT_PIN 2

NaviguiderCompass compass(INTERRUPT_PIN);

void setup() {
    SerialUSB.begin(9600);
    while (!SerialUSB);

    SerialUSB.println("Starting NaviGuider initialization...");

    if (!compass.begin()) {
        SerialUSB.println("Failed to initialize NaviGuider Compass!");
        while (1);
    }

    SerialUSB.println("NaviGuider initialized successfully.");
}


void loop() {

    // Read Sensors, and Service Interrupts every loop
    compass.readSensors();

    // Output Heading Pitch Roll and Yaw Rate
    SerialUSB.print("Heading: "); SerialUSB.print(compass.getHeading()); 
    SerialUSB.print("    Pitch: "); SerialUSB.print(compass.getPitch()); 
    SerialUSB.print("    Roll: "); SerialUSB.print(compass.getRoll()); 
    SerialUSB.print("    Yaw Rate: "); SerialUSB.println(compass.getYawRate());

    // There's no need for a delay
    delay(100);
}