# ESP32 Mini Drone 🚁

This project is a compact quadcopter drone built using an **ESP32 microcontroller as the flight controller**. The drone is designed as a lightweight experimental platform for learning drone flight control, sensor integration, and embedded systems.

## Features
- ESP32-based flight controller
- Compact custom drone frame
- MPU6050 IMU for orientation and stabilization
- Lightweight brushed motor propulsion system
- Designed for experimentation with mini drones

## Hardware Used
- ESP32 development board  
- MPU6050 IMU sensor  
- 4 × 8520 brushed DC motors  
- 65 mm propellers  
- Custom 100 mm wheelbase frame (designed in Onshape)  
- 1s LiPo battery  (minimum 30A rated)

## Frame Design
The drone frame was **custom designed using Onshape** with a **100 mm wheelbase**, making it compact and lightweight for mini drone experiments.

## Working
The ESP32 reads motion and orientation data from the MPU6050 IMU. Using this data, the flight controller adjusts motor speeds to maintain stability and control the drone during flight.

## Applications
- Learning drone flight control
- Embedded systems experimentation
- Mini drone prototyping
- Robotics projects

## License
This project is open-source and intended for educational and experimental use.
