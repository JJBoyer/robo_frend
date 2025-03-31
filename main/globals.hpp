/*

Filename: globals.hpp
Author: Jacob Boyer
Description: Header file for globals.cpp. Defines structs used
in transmitting states, waypoints, and paths between members in
the autonomy stack.

*/

#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include <cmath>
#include <memory>
#include <vector>
#include <queue>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Degree to radian conversion
const float deg_to_rad = M_PI / 180.0f;

// Declare a state_t struct for use in state estimation
typedef struct {
    float posX; // position along global x-axis
    float posY; // position along global y-axis
    float th;   // angle of robot y-axis from global y-axis
    float velX; // velocity along global x-axis
    float velY; // velocity along global y-axis
    float w;    // angular velocity about z-axis
    float acc;  // acceleration along robot y-axis
} state_t;

// Declare a waypoint_t struct for use in the waypoint vector
typedef struct {
    float x;  // position along global x-axis
    float y;  // position along global y-axis
    float th; // (degrees) angle of robot y-axis from global y-axis
} Pose2D_t;

// Declare a pathPoint_t struct for use in the path vector
struct pathPoint_t {
    float x;
    float y;

    pathPoint_t operator+(const pathPoint_t& other) const { return {x + other.x, y + other.y}; }
    pathPoint_t operator-(const pathPoint_t& other) const { return {x - other.x, y - other.y}; }
    pathPoint_t operator*(float scalar) const { return {x * scalar, y * scalar}; }
};

// Declare a motorOut_t struct for use in sending outputs to motors
typedef struct {
    int left;
    int right;
} motorOut_t;

// Declare external global pointers for thread communication
extern std::unique_ptr<motorOut_t> pduty;
extern std::unique_ptr<double> pdist;
extern std::unique_ptr<state_t> pstate;
extern std::unique_ptr<std::queue<Pose2D_t>> pwaypt;
extern std::unique_ptr<std::vector<pathPoint_t>> ppath;

// Declare external mutexes to access global pointers
extern SemaphoreHandle_t dutyMutex;
extern SemaphoreHandle_t distMutex;
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t wayMutex;
extern SemaphoreHandle_t pathMutex;

// Declare system parameters
extern const float velocityTarget;
extern const float wheelBaseWidth;

void initGlobals();

#endif