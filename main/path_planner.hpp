#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "globals.hpp"

void getStart();
void getTarget();
void computeControlPoints(const Pose2D_t& start, const Pose2D_t& end, float d, pathPoint_t& P0, 
                          pathPoint_t& P1, pathPoint_t& P2, pathPoint_t& P3);
void generateBezierPath(const Pose2D_t& start, const Pose2D_t end, float d, int num_points = 50);
pathPoint_t bezierPoint(float t, const pathPoint_t& P0, const pathPoint_t& P1, const pathPoint_t& P2, const pathPoint_t& P3);

#endif