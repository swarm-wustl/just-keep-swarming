#include "pid.h"

// TODO: test Kd
// TODO: add Ki

// Angular constsants
#define Kp_angular 2.0
#define Kd_angular 0.5

// Linear constants
#define Kp_linear 1.0
#define Kd_linear 0.3

static int32_t angular_time_prev = -1;
static double angular_error_prev = 0.0;

static int32_t linear_time_prev = -1;
static double linear_error_prev = 0.0;

double angular_error_to_velocity(double error, int32_t time_curr) {
    double derivative;

    // If this is the first packet, don't include a derivative term
    // Otherwise, calculate the derivative based on current and previous values
    if (angular_time_prev == -1) {
        derivative = 0;
    } else {
        derivative = (error - angular_error_prev) / (time_curr - angular_time_prev);
    }

    angular_time_prev = time_curr;
    angular_error_prev = error;

    return (Kp_angular * error) + (Kd_angular * derivative);
}

double linear_error_to_velocity(double error, int32_t time_curr) {
    double derivative;

    // If this is the first packet, don't include a derivative term
    // Otherwise, calculate the derivative based on current and previous values
    if (linear_time_prev == -1) {
        derivative = 0;
    } else {
        derivative = (error - linear_error_prev) / (time_curr - linear_time_prev);
    }

    linear_time_prev = time_curr;
    linear_error_prev = error;

    return (Kp_linear * error) + (Kd_linear * derivative);
}