# robo_frend – ESP32 Autonomous Navigation Platform

## Overview
**robo_frend** is a real-time autonomous robot built on the ESP32-S3 platform using C++ and FreeRTOS. 
The project explores onboard autonomy through modular task-based control, sensor fusion, and trajectory tracking — all implemented on a mobile robot designed for dynamic navigation. 
Upon completion of the first version, robo_frend will be able to navigate to a user-entered waypoint with a specified heading on an open grid (i.e. [5, 3, 45deg]).

## Key Features
- ✅ Real-time task coordination using FreeRTOS
- ✅ IMU-based 2D state estimation
- ✅ Cubic Bézier curve path planning
- 🔄 Pure pursuit control *(in progress / tuning)*
- Ultrasonic collision avoidance *(planned)*
- MQTT communication for remote waypoint input *(planned)*

## System Architecture

The robot’s autonomy stack is structured around modular tasks managed by FreeRTOS. 
Each major function of the system—motion, sensing, communication, and control—is isolated into its own task to ensure real-time responsiveness and maintainable concurrency.

- **State Estimation Task**  
  Continuously reads IMU data over I2C to estimate the robot’s 2D position and orientation using integrated acceleration and angular velocity. This state estimate forms the basis for control and path planning decisions.

- **Path Planning Task**  
  Generates smooth trajectories between waypoints using cubic Bézier curves. These paths are sampled into target points that are passed to the trajectory tracker.

- **Path Following Control Task**  
  Implements a pure pursuit controller to calculate the linear and angular velocity needed to reach the next target along the path. Converrts velocity command to   
  directional PWM outputs.

- **Motor Output Task**
  Receives PWM outputs from controller and applies them to the motors, adjusting motor speed and direction using an L298N motor driver.

- **Communication Task (Planned)**  
  Will handle MQTT-based communication with a base station, allowing remote waypoint commands to be sent to the robot over Wi-Fi.

- **Collision Avoidance (Planned)**  
  Will use an ultrasonic distance sensor to detect nearby obstacles and trigger evasive maneuvers or controller overrides.

All tasks are synchronized using FreeRTOS scheduling primitives (e.g., queues, mutexes), and the system is designed for expandability and real-time performance in embedded environments.

## Tools & Technologies
- **Hardware:** ESP32-S3, DC motors, MPU6050 IMU, HC-SR04 ultrasonic sensor
- **Software:** ESP-IDF, FreeRTOS, C++
- **Protocols:** I2C, PWM, UART, MQTT
- **Development:** Git, GitHub, WSL

## Disclaimer
This is an active development project. 
The repo reflects a work-in-progress implementation as part of a larger effort to develop onboard real-time autonomy on embedded platforms. 
Check back for updates as new functionality is added and tested.

## License

This project is licensed under the [MIT License](LICENSE).

