

## Info



## License

 BSD license, all text above must be included in any redistribution.

# Naviguider_Compass_I2C
Library for interfacing with the Naviguider I2C Compass module.



1. Introduction
If you've ever worked with sensors, displays, or pretty much any embedded device, you've probably come across I2C (Inter-Integrated Circuit). It’s one of the most common ways for microcontrollers to talk to peripherals. But while using I2C libraries is pretty straightforward, writing a driver from scratch is another story. That’s what we’re going to dive into—how to build an I2C driver for a real-world sensor, step by step.

For this, I worked on developing a driver for the Naviguider Compass, a sensor module that uses the SENtral-A2 fusion processor. The goal? Get reliable heading, orientation, and motion data from the device over I2C and make it easy to use in embedded projects.

To do this, I had to dig through two key datasheets:

The Naviguider Compass datasheet, which lays out the sensor's capabilities, register map, and data formats.
The SENtral-A2 datasheet, which explains how the sensor fusion processor works, how it manages sensor data, and how to configure it properly.
Along the way, I also referenced some example code, but as always, example code only gets you so far. To really understand what’s going on and build something robust, you have to break things down and figure out how the sensor actually expects data to be read and written. That’s exactly what we’re going to do.

