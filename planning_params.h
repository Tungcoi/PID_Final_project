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
//#define P_NUM_PATHS 1                  // TODO - Num of paths (goals)
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

#define P_STOP_THRESHOLD_DISTANCE \
  P_LOOKAHEAD_MIN / P_NUM_POINTS_IN_SPIRAL * 2  // Threshold distance to stop (m)

constexpr std::array<float, 3> CIRCLE_OFFSETS = {-1.0, 1.0, 3.0};  //  // Offsets for collision circles (m)
constexpr std::array<float, 3> CIRCLE_RADII = {1.5, 1.5, 1.5};     // // Radii of collision circles (m)

constexpr double dt = 0.05;  // Time step for motion updates
// Standard deviation parameters for x, x_dot, x_double_dot
// to generate appropriate perturbed goals. EGO REF FRAME
constexpr std::array<float, 3> SIGMA_X = {4, 1.0, 2.0};  // Variance for x position, velocity, acceleration

// Standard devaition parameters for y, y_dot, y_double_dot
// to generate appropriate perturbed goals. EGO REF FRAME
constexpr std::array<float, 3> SIGMA_Y = {0.5, 1.0, 0.5};  // Variance for y position, velocity, acceleration

// Standard deviation parameters for yaw, yaw_rate, yaw_acceleration
constexpr std::array<float, 3> SIGMA_YAW = {0.17, 1.0, 1.0};  // Variance for yaw, yaw rate, yaw acceleration

// Standard deviation for time (as in the time
// taken to finish the maneuver)
constexpr double SIGMA_T = 0.5;

// Comfort-related constraints on jerk and acceleration
// This would be the filtered jerk over one sec
constexpr double CONFORT_MAX_LAT_JERK = 0.9;               // Maximum lateral jerk (m/s^3)
constexpr double CONFORT_MAX_LON_JERK = 1.5;               // Maximum longitudinal jerk (m/s^3)
constexpr double CONFORT_ACCUM_LON_JERK_IN_ONE_SEC = 3.0;  // Accumulated longitudinal jerk in one second (m/s^3)
constexpr double CONFORT_ACCUM_LAT_JERK_IN_ONE_SEC = 2.0;  // Accumulated lateral jerk in one second (m/s^3)

constexpr double CONFORT_ACCUM_LON_ACC_IN_ONE_SEC = 1.0;  // Accumulated longitudinal acceleration in one second (m/s^2)
constexpr double CONFORT_ACCUM_LAT_ACC_IN_ONE_SEC = 0.6;  // Accumulated lateral acceleration in one second (m/s^2)

constexpr double CONFORT_MAX_LON_ACCEL = 3.0;  // Maximum longitudinal acceleration (m/s^2)
constexpr double CONFORT_MAX_LAT_ACCEL = 1.0;  // Maximum lateral acceleration (m/s^2)

// Time limits for maneuver execution
constexpr double MIN_MANEUVER_TIME = dt * 10;  // Minimum steps for maneuver time
constexpr double MAX_MANEUVER_TIME = dt * 75;  // Maximum steps for maneuver time
