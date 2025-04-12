/*



*/

#ifndef STATE_ESTIMATION_HPP
#define STATE_ESTIMATION_HPP

#include "globals.hpp"
#include "imu.hpp"
#include "encoder.hpp"

// Function to run a Kalman filter and estimate robot state
void estimateState(mpu6050_handle_t& mpu_sensor);

// Function to apply kalman filter to state estimation
void kalman(state_t& current, state_t& past, float& dt);

#endif