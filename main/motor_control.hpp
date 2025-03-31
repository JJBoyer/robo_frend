/*

Filename: motor_control.hpp
Author: Jacob Boyer
Description: Header file for motor_control.hpp

*/

#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "driver/ledc.h"
#include "globals.hpp"

// Function to initialize motor control pins
void initMotors();

// Function to write PWM signal to motors
void forward();

// Define PWM hardware pin
#define LEFT_PWM_PIN GPIO_NUM_2    // Connects to ENA on driver
#define RIGHT_PWM_PIN GPIO_NUM_42  // Connects to ENB on driver

// Define motor direction hardware pins
#define LEFT_FORWARD GPIO_NUM_7   // Connects to IN1 on driver
#define LEFT_REVERSE GPIO_NUM_6   // Connects to IN2 on driver
#define RIGHT_REVERSE GPIO_NUM_5  // Connects to IN3 on driver
#define RIGHT_FORWARD GPIO_NUM_4  // Connects to IN4 on driver

#endif