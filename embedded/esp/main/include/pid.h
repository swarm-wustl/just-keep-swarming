#pragma once

#include <std_msgs/msg/int32.h>

// Distance thresholds
#define DISTANCE_TOLERANCE 0.05 // meters
#define ANGLE_TOLERANCE 0.05    // rad

// Stepping functions
double angular_error_to_velocity(double error, int32_t time_curr);
double linear_error_to_velocity(double error, int32_t time_curr);
