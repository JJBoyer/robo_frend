/*

Filename: pure_pursuit.hpp
Author: Jacob Boyer
Description: Header file for pure_pursuit.cpp

*/

#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "globals.hpp"

void getState();
void getPath();
void sendToMotors(motorOut_t duty);
void purePursuitControl(const state_t& state, const std::vector<pathPoint_t>& path, float lookaheadDist);
pathPoint_t findLookaheadPoint(const state_t& state, const std::vector<pathPoint_t>& path, float L);
float computeCurvature(const state_t& state, const pathPoint_t& lookahead);
int mapVelocityToPWM(float v);

#endif