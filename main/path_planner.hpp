/*

Filename: path_planner.hpp
Author: Jacob Boyer
Description: Header file for path_planner.cpp

*/

#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "globals.hpp"

// Function to retrieve the global state
void getStart();

// Function to retrieve the next waypoint
void getTarget();

// Function to compute control points for path planning
void computeControlPoints(const Pose2D_t& start, const Pose2D_t& end, float d, pathPoint_t& P0, 
                          pathPoint_t& P1, pathPoint_t& P2, pathPoint_t& P3);

// Function to create a vector of points to define the path
void generateBezierPath(const Pose2D_t& start, const Pose2D_t end, float d, int num_points = 50);

// Function to create each individual path point
pathPoint_t bezierPoint(float t, const pathPoint_t& P0, const pathPoint_t& P1, const pathPoint_t& P2, const pathPoint_t& P3);

#endif