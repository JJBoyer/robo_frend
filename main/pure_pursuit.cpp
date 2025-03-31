/*

Filename: pure_pursuit.cpp
Author: Jacob Boyer
Description: The pure pursuit controller takes in the current
state and the planned path, then uses those values to find a
lookahead point along the path to produce motor outputs that
steer the robot along its planned path.

*/

#include "esp_log.h"
#include "globals.hpp"
#include "mutex_guard.hpp"
#include "pure_pursuit.hpp"

using namespace std;

// Initialize a motorOut_t struct to receive controller output
motorOut_t duty = {
    .left = 0,
    .right = 0
};

// Declare a state_t struct to contain the current state when measured
state_t state;
vector<pathPoint_t> path;

// Define lookahead distance
float lookaheadDist = 0.25f;

// Flag for whether the path has been imported for pure pursuit
bool pathMapped = false;

/* getState:
    Reads pstate and saves the current state
    for use in the controller.
*/
void getState(){

    MutexGuard lock(stateMutex);

    // Save current value at pstate to local state
    state = *pstate;

}

/* getPath:
    Reads ppath and saves it to a local vector to
    find pure pursuit lookahead points.
*/
void getPath(){

    MutexGuard lock(pathMutex);

    // Save global path vector at ppath to local path vector
    path = *ppath;

}

/* sendToMotors:
    Takes the calculated motor duty cycles saved
    to duty and exports them to pduty for use in
    motor driving.
*/
void sendToMotors(motorOut_t duty){
    MutexGuard lock(dutyMutex);
    *pduty = duty;
}

/* purePursuitControl:
    Takes the current state, the planned path, and the lookahead
    distance and produces motor outputs to achieve the path curvature
    and motor power to follow the path.
*/
void purePursuitControl(){
    
    // Get path vector if not yet updated
    if(!pathMapped){
        getPath();
        pathMapped = true;
    }

    // Update state each time
    getState();

    // Find the lookahead point and curvature for next command
    pathPoint_t lookahead = findLookaheadPoint(state, path, lookaheadDist);
    float curvature = computeCurvature(state, lookahead);

    // Calculate next command
    float v_l = velocityTarget - (curvature * velocityTarget * wheelBaseWidth / 2.0f);
    float v_r = velocityTarget + (curvature * velocityTarget * wheelBaseWidth / 2.0f);

    // Place PWM values in duty struct
    duty.left  = mapVelocityToPWM(v_l);
    duty.right = mapVelocityToPWM(v_r);

    // Export duty struct to pduty
    sendToMotors(duty);
}

/* findLookaheadPoint:
    Given the current state, the path vector, and
    a lookahead distance, this function finds and
    returns the next lookahead point for the controller.
*/
pathPoint_t findLookaheadPoint(const state_t& state, const std::vector<pathPoint_t>& path, float L){

    // Check path vector for the nearest point beyond the lookahead distance
    for(const auto& pt : path){

        float dx = pt.x - state.posX;
        float dy = pt.y - state.posY;
        float dist = sqrt(dx*dx + dy*dy);

        if(dist >= L){
            return pt;
        }
    }
    // If no point is beyond the lookahead distance, return the final point in the path vector
    return path.back();
}

/* computeCurvature:
    Transforms the next lookahead point from the world frame
    into the robot frame and uses the pure pursuit formula to
    calculate the curvature for the controller output.
*/
float computeCurvature(const state_t& state, const pathPoint_t& lookahead){

    // Solve for the deltas in the world frame
    float dx = lookahead.x - state.posX;
    float dy = lookahead.y - state.posY;

    // Use the current heading and deltas to transform the lookahead point into the robot frame from world frame
    float heading = state.th;
    float localX = cos(-heading * deg_to_rad) * dx - sin(-heading * deg_to_rad) * dy;
    float localY = sin(-heading * deg_to_rad) * dx + cos(-heading * deg_to_rad) * dy;

    // Avoid cases that would divide by zero
    if(localY == 0) return 0;

    // Apply the pure pursuit formula to return the curvature for control
    return 2 * localX / (localX * localX + localY * localY);
}

/* mapVelocityToPWM:
    Takes in the velocity commands of the pure pursuit controller
    and converts from velocity to PWM for use in motor control.
*/
int mapVelocityToPWM(float v){
    const float max_speed = 0.3f;  // Max expected speed
    const int pwm_min = 512;
    const int pwm_max = 1023;

    // Map the range of possible speeds onto a range between pwm_min and pwm_max
    if(v <= 0.1f) return 0;  // dead zone
    int pwm = pwm_min + static_cast<int>((v / max_speed) * (pwm_max - pwm_min));
    return clamp(pwm, 0, pwm_max);
}