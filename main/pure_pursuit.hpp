/*

Filename: pure_pursuit.hpp
Author: Jacob Boyer
Description: Header file for pure_pursuit.cpp

*/

#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "globals.hpp"

// Function to retrieve the global state
void getState();

// Function to retrieve the planned path
void getPath();

// Wrapper to simplify writting to pduty
void sendToMotors(motorOut_t duty);

// Function to apply pure pursuit control
void purePursuitControl();

// Function to find nearest path point beyond lookahead distance
pathPoint_t findLookaheadPoint(const state_t& state, const std::vector<pathPoint_t>& path, float L);

// Function to compute desired motor speeds
float computeCurvature(const state_t& state, const pathPoint_t& lookahead);

// Function to convert motor speeds to pwm duty cycles for motor output
int mapVelocityToPWM(float v);

#endif