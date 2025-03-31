/*

Filename: path_planner.cpp
Author: Jacob Boyer
Description: Receives a waypoint from the waypoint queue,
compares it to the state provided by the state estimator,
and uses a cubic Bezier curve to generate a path to feed
the pure pursuit tracker. Outputs the path vector to the
ppath global pointer.

*/

#include "path_planner.hpp"
#include "mutex_guard.hpp"

using namespace std;

Pose2D_t start;
Pose2D_t target;

/* getStart:
    Pulls the current state from pstate and saves it
    to start for use in generating the Bezier curve.
*/
void getStart(){

    state_t current;
    {
        MutexGuard lock(stateMutex);
        current = *pstate;
    }

    start.x  = current.posX;
    start.y  = current.posY;
    start.th = current.th;

}

/* getTarget:
    Pulls the next waypoint off the queue and saves it
    to target for use in generating the Bezier curve.
*/
void getTarget(){

    // Take wayMutex to secure pwaypt
    MutexGuard lock(wayMutex);

    // Check if waypoint is available
    if((*pwaypt).empty()){
        printf("No waypoints available!");
        return;
    }
    
    // Copy first element in waypoint queue
    target = (*pwaypt).front();

    // Remove element from queue
    (*pwaypt).pop();

}

/* computeControlPoints:
    Takes in start and target poses, as well as a user-defined d-value,
    then computes the control points necessary for generating the curve.
*/
void computeControlPoints(const Pose2D_t& start, const Pose2D_t& target, float d, pathPoint_t& P0, 
                          pathPoint_t& P1, pathPoint_t& P2, pathPoint_t& P3) {

    // Update current pose and target pose
    getStart();
    getTarget();

    // Calculate heading vectors
    pathPoint_t heading_start = {cos(start.th * deg_to_rad), sin(start.th * deg_to_rad)};
    pathPoint_t heading_target = {cos(target.th * deg_to_rad), sin(target.th * deg_to_rad)};

    // Calculate control points
    P0 = {start.x, start.y};
    P1 = P0 + heading_start * d;
    
    P3 = {target.x, target.y};
    P2 = P3 - heading_target * d;

}

/* generateBezierPath:
    Takes in the start and target poses, d, and a number of points
    to define the resolution of the curve. Computes the control points
    and applies the cubic Bezier curve equation for each point to 
    generate the path. The points are saved to a vector, which is then 
    exported to ppath for use in pure pursuit.
*/
void generateBezierPath(const Pose2D_t& start, const Pose2D_t target, float d, int num_points){

    // Compute the control points based on desired start and end poses
    pathPoint_t P0, P1, P2, P3;
    computeControlPoints(start, target, d, P0, P1, P2, P3);

    // Create path vector and allocate
    vector<pathPoint_t> path;
    path.reserve(num_points);

    // Create path points
    for (int i = 0; i <= num_points; i++){
        float t = static_cast<float>(i) / num_points;
        path.push_back(bezierPoint(t, P0, P1, P2, P3));
    }

    // Export path to global path pointer
    {
        MutexGuard lock(pathMutex);
        *ppath = path;
    }
}

/* bezierPoint:
    Takes in the control points and path progress variable,
    then applies the cubic Bezier curve equation for that point.
*/
pathPoint_t bezierPoint(float t, const pathPoint_t& P0, const pathPoint_t& P1, const pathPoint_t& P2, const pathPoint_t& P3){

    // Calculate u for use in the Bezier equation
    float u = 1.0f - t;

    // Use the Bezier equation to calculate the point and return it
    return P0 * (u * u * u) + P1 * (3 * u * u * t) + P2 * (3 * u * t * t) + P3 * (t * t * t);

}