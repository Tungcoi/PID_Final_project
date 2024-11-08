/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file planning_param.h
 **/

#pragma once

#include <array>

// Planning Constants
#ifdef CODE_P6
#define P_NUM_PATHS 7
#define P_LOOKAHEAD_MIN 8.0            // m
#define P_LOOKAHEAD_MAX 20.0           // m
#define P_LOOKAHEAD_TIME 1.5           // s
#define P_GOAL_OFFSET 1.0              // m
#define P_ERR_TOLERANCE 0.1            // m
#define P_TIME_GAP 1.0                 // s
#define P_MAX_ACCEL 1.5                // m/s^2
#define P_SLOW_SPEED 1.0               // m/s
#define P_SPEED_LIMIT 3.0              // m/s
#define P_STOP_LINE_BUFFER 0.5         // m
#define P_STOP_THRESHOLD_SPEED 0.02    // m/s
#define P_REQ_STOPPED_TIME 1.0         // secs
#define P_LEAD_VEHICLE_LOOKAHEAD 20.0  // m
#define P_REACTION_TIME 0.25           // secs
#define P_NUM_POINTS_IN_SPIRAL 20
#elif defined(CODE_SELF)
#define P_NUM_PATHS 7   // Number of lateral offset paths from the center-line goal
// Lookahead distance and time for path planning
#define P_LOOKAHEAD_MIN 8.0            // m
#define P_LOOKAHEAD_MAX 20.0           // m
#define P_LOOKAHEAD_TIME 1.5           // s
// Offset for generating alternate goals
// #define P_GOAL_OFFSET 1.0              // m
#define P_GOAL_OFFSET 1.5              // Lateral offset distance for alternate goals (m)

// Tolerance and thresholds
#define P_ERR_TOLERANCE 0.1            // Position tolerance (m)
#define P_TIME_GAP 1.0                 // Safe time gap for following vehicles (s)
#define P_MAX_ACCEL 1.5                // Maximum acceleration (m/s^2)
#define P_SLOW_SPEED 1.0               // Slow speed near stop points (m/s)
#define P_SPEED_LIMIT 3.0              // Speed limit (m/s)
#define P_STOP_LINE_BUFFER 0.5         // Buffer distance behind stop line (m)
#define P_STOP_THRESHOLD_SPEED 0.02    // Speed threshold to consider the vehicle as stopped (m/s)
#define P_REQ_STOPPED_TIME 1.0         // Required stop time at stop signs (s)
#define P_LEAD_VEHICLE_LOOKAHEAD 20.0  // Lookahead distance for lead vehicle (m)
#define P_REACTION_TIME 0.25           // Reaction time for following lead vehicles (s)
//#define P_NUM_POINTS_IN_SPIRAL 2       // TODO - Num of points in the spiral
#define P_NUM_POINTS_IN_SPIRAL 150     // Number of points used to discretize the cubic spiral

#else
#define P_NUM_PATHS 5                  // Number of deviations to enumerate
// Minimum lookhead distance (m)
#define P_LOOKAHEAD_MIN 8.0
// Maximum lookahead distance (m)
#define P_LOOKAHEAD_MAX 20.0
// Lookahead time (s)
#define P_LOOKAHEAD_TIME 1.5
// Goal-offset distance (m)
#define P_GOAL_OFFSET 1.0
// Error tolerance (m)
#define P_ERR_TOLERANCE 0.1
// Time-gap (s) to enforce
#define P_TIME_GAP 1.0
// Maximum permitted accelertion (m/s^2)
#define P_MAX_ACCEL 1.5
// Speed (m/s) to use for slow manoeuvre
#define P_SLOW_SPEED 1.0
// Speed limit (m/s) to set in `FOLLOW_LANE` state
#define P_SPEED_LIMIT 3.0
// Distance (m) to buffer from stopping point (stay behind)
#define P_STOP_LINE_BUFFER 0.5
// Maximum speed (m/s) to set for stopping manoeuvre
#define P_STOP_THRESHOLD_SPEED 0.02
// Minimum required stopping time (s)
#define P_REQ_STOPPED_TIME 1.0
// Lookahead distance (m) to use for lead vehicle
#define P_LEAD_VEHICLE_LOOKAHEAD 20.0
// Minimum required reaction time (s)
#define P_REACTION_TIME 0.25
// Number of waypoints to use in polynomial spiral (path)
// CANDO: Modify the number of waypoints to use in each path
// NOTE: Needs to be sufficiently large to avoid compile error
#define P_NUM_POINTS_IN_SPIRAL 25

#endif


#define P_STOP_THRESHOLD_DISTANCE \
  P_LOOKAHEAD_MIN / P_NUM_POINTS_IN_SPIRAL * 2  // m

constexpr std::array<float, 3> CIRCLE_OFFSETS = {-1.0, 1.0, 3.0};  // m
constexpr std::array<float, 3> CIRCLE_RADII = {1.5, 1.5, 1.5};     // m

constexpr double dt = 0.05;
// Standard devaition parameters for x, x_dot, x_double_dot
// to generate appropriate perturbed goals. EGO REF FRAME
constexpr std::array<float, 3> SIGMA_X = {4, 1.0, 2.0};

// Standard devaition parameters for y, y_dot, y_double_dot
// to generate appropriate perturbed goals. EGO REF FRAME
constexpr std::array<float, 3> SIGMA_Y = {0.5, 1.0, 0.5};

constexpr std::array<float, 3> SIGMA_YAW = {0.17, 1.0, 1.0};

// Standard deviation for time (as in the time
// taken to finish the maneuver
constexpr double SIGMA_T = 0.5;

// This would be the filtered Jerk over one sec
constexpr double CONFORT_MAX_LAT_JERK = 0.9;               // m/s3
constexpr double CONFORT_MAX_LON_JERK = 1.5;               // m/s3
constexpr double CONFORT_ACCUM_LON_JERK_IN_ONE_SEC = 3.0;  // m/s3
constexpr double CONFORT_ACCUM_LAT_JERK_IN_ONE_SEC = 2.0;  // m/s3

constexpr double CONFORT_ACCUM_LON_ACC_IN_ONE_SEC = 1.0;  // m/s2
constexpr double CONFORT_ACCUM_LAT_ACC_IN_ONE_SEC = 0.6;  // m/s2

constexpr double CONFORT_MAX_LON_ACCEL = 3.0;  // m/s2
constexpr double CONFORT_MAX_LAT_ACCEL = 1.0;  // m/s2

constexpr double MIN_MANEUVER_TIME = dt * 10;  // min steps
constexpr double MAX_MANEUVER_TIME = dt * 75;  // max steps