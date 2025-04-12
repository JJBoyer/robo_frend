/*



*/

#include <Eigen/Dense>
#include "esp_log.h"
#include "state_estimation.hpp"
#include "mutex_guard.hpp"

using namespace std;

/* estimateState:
    Collects accelerometer data from IMU and
    uses trapezoidal integration to estimate the
    position and velocity components of the system
    state. Exports the estimated state to pstate.
*/
void estimateState(mpu6050_handle_t& mpu_sensor){

    // Initialize state_t for current state
    static state_t current = {
        .posX = 0.0f,
        .posY = 0.0f,
        .th   = 0.0f,
        .velX = 0.0f,
        .velY = 0.0f,
        .w    = 0.0f,
        .acc  = 0.0f
    };

    // Initialize state_t for previous state
    static state_t past = {
        .posX = 0.0f,
        .posY = 0.0f,
        .th   = 0.0f,
        .velX = 0.0f,
        .velY = 0.0f,
        .w    = 0.0f,
        .acc  = 0.0f
    };

    // Calculate dt for this loop
    static int64_t last_time = 0;
    int64_t now = esp_timer_get_time();  // microseconds
    float dt = (now - last_time) / 1e6;  // seconds
    last_time = now;
    if (dt <= 0.0f || dt > 1.0f) return; // if timing is invalid, skip this cycle

    // Collect data for this timestep
    measureAccel(mpu_sensor);
    measureGyro(mpu_sensor);
    measureVelocity();

    // Apply Kalman filter to estimate the current state
    kalman(current, past, dt);

    // Export new state estimate to global state variable
    {
        MutexGuard lock(stateMutex);
        pstate.reset(new state_t(current));
    }

    past = current;
}

void kalman(state_t& current, state_t& past, float& dt){

    // State Vector
    static Eigen::Matrix<float, 7, 1> X = Eigen::Matrix<float, 7, 1>::Zero();
    X << past.posX, past.posY, past.th, past.velX, past.velY, past.w, past.acc;

    // State Transition Matrix
    static Eigen::Matrix<float, 7, 7> A = Eigen::Matrix<float, 7, 7>::Identity();
    A.row(0) << 1,  0,  0, dt,  0,  0,  0.5f * cos(past.th) * dt * dt;
    A.row(1) << 0,  1,  0,  0, dt,  0,  0.5f * sin(past.th) * dt * dt;
    A.row(2) << 0,  0,  1,  0,  0, dt,  0;
    A.row(3) << 0,  0,  0,  1,  0,  0,  cos(past.th)*dt;
    A.row(4) << 0,  0,  0,  0,  1,  0,  sin(past.th)*dt;
    A.row(5) << 0,  0,  0,  0,  0,  1,  0;
    A.row(6) << 0,  0,  0,  0,  0,  0,  1;

    // State estimate uncertainty matrix
    static Eigen::Matrix<float, 7, 7> P = Eigen::Matrix<float, 7, 7>::Identity();
    P(0, 0) = 0.01f;    // posX  (integrated)
    P(1, 1) = 0.01f;    // posY  (integrated)
    P(2, 2) = 0.01f;    // theta (integrated)
    P(3, 3) = 0.001f;   // velX  (calculated)
    P(4, 4) = 0.001f;   // velY  (calculated)
    P(5, 5) = 0.0001f;  // omega (measured)
    P(6, 6) = 0.1f;     // acc   (measured)

    // State model uncertainty (drift, slippage, etc.) matrix
    static Eigen::Matrix<float, 7, 7> Q = Eigen::Matrix<float, 7, 7>::Identity();
    Q(0, 0) = 0.01f;   // posX
    Q(1, 1) = 0.01f;   // posY
    Q(2, 2) = 0.001f;  // theta
    Q(3, 3) = 0.1f;    // velX
    Q(4, 4) = 0.1f;    // velY
    Q(5, 5) = 0.01f;   // omega
    Q(6, 6) = 0.5f;    // acc

    // Measurement model matrix
    static Eigen::Matrix<float, 3, 7> H = Eigen::Matrix<float, 3, 7>::Zero();
    H.row(0) << 0, 0, 0, 1.0f, 0, 0, 0;  // encoder velX to state velX
    H.row(1) << 0, 0, 0, 0, 1.0f, 0, 0;  // encoder velY to state velY
    H.row(2) << 0, 0, 0, 0, 0, 1.0f, 0;  // gyro omega to state omega

    // Measurement uncertainty matrix
    static Eigen::Matrix<float, 3, 3> R = Eigen::Matrix<float, 3, 3>::Zero();
    R(0, 0) = 0.05f;  // velX  (encoders)
    R(1, 1) = 0.05f;  // velY  (encoders)
    R(2, 2) = 0.01f;  // omega (gyro)

    // Kalman gain matrix
    static Eigen::Matrix<float, 7, 3> K = Eigen::Matrix<float, 7, 3>::Zero();

    // Measurement vector
    static Eigen::Matrix<float, 3, 1> Z = Eigen::Matrix<float, 3, 1>::Zero();  // Measurement vector
    Z << wheel.avg * cos(past.th), wheel.avg * sin(past.th), gyro.gyro_z;

    // Identity matrix
    Eigen::Matrix<float, 7, 7> I = Eigen::Matrix<float, 7, 7>::Identity();

    // Predict next state
    X = A * X;

    // Update P matrix
    P = A * P * A.transpose() + Q;

    // Calculate kalman gain
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Filtered state estimate: X = x + K * y, y = Z - H * X
    X = X + K * (Z - H * X);

    // Refine P matrix
    P = (I - K * H) * P;

    // Save new state estimate to current
    current.posX = X(0);
    current.posY = X(1);
    current.th   = X(2);
    current.velX = X(3);
    current.velY = X(4);
    current.w    = X(5);
    current.acc  = X(6);

}