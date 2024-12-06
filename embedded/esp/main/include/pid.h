#pragma once

// Distance thresholds
#define DISTANCE_TOLERANCE 0.05 // meters
#define ANGLE_TOLERANCE 0.05    // rad

// Stepping functions
double angular_error_to_velocity(double error);
double linear_error_to_velocity(double error);
