# Naviguider_Compass_I2C

An Arduino library for interfacing with the PNI NaviGuider I2C Compass. This library simplifies communication with the NaviGuider Compass, handling register reads/writes and sensor data parsing.

## About This Project

This library was developed while integrating the NaviGuider Compass into an ROV sensor package. The process of writing the I2C device driver was documented in an article, which you can read here:

[How to Write an I2C Device Driver: Case Study - The NaviGuider Compass](#) *(https://www.linkedin.com/pulse/how-write-i2c-device-driver-case-study-naviguider-compass-travers-7e0uc/?trackingId=4u84ngkF%2F4s1BDLPCgidVw%3D%3D)*

## Installation

To install this library:

1. Download the latest release from the [Releases](https://github.com/MarineAppliedResearch/Naviguider_Compass_I2C/releases) page.
2. Extract the folder and place it inside your Arduino libraries directory (`Documents/Arduino/libraries/`).
3. Open Arduino IDE and include the library in your sketch:

   ```cpp
   #include <Naviguider_Compass_I2C.h>
   ```

## Usage

A basic example of using the NaviGuider Compass:

```cpp
#include <Wire.h>
#include <Naviguider_Compass_I2C.h>

NaviguiderCompass compass;

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
```

For more details, check out the [examples](examples/) directory.

## Contributing

Contributions are welcome! If you'd like to contribute:

- Fork the repository.
- Create a new branch for your feature or bug fix.
- Commit your changes with meaningful commit messages.
- Open a pull request with a clear description of your changes.

### Coding Standards

- Use **clear and concise comments** to explain functionality.
- Follow **consistent indentation** (1 tab per indentation level).
- Function and variable names should be **descriptive and use camelCase**.
- When adding new features, ensure they are **well-documented and tested**.

## License

This project is licensed under the BSD License. See the [LICENSE](license.txt) file for details.

---

If you have any questions, feel free to open an issue or reach out!

## Version History

### [1.0.1] - 2025-02-05
- Initial release of the **Naviguider_Compass_I2C** library.
- Implements basic I2C communication with the **PNI NaviGuider Compass**.
- Supports reading orientation data from the FIFO buffer.
- Provides a simple API for initializing the compass and retrieving heading data.
- Compatible with the Arduino ecosystem.

---

### Upcoming Features
Planned improvements and future updates:
- Add support for more advanced sensor configurations.
- Improve error handling and debugging messages.
- Expand documentation with more usage examples.