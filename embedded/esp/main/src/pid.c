#include "pid.h"

// TODO: add Kd, Ki

// Angular constsants
#define Kp_angular 2.0

// Linear constants
#define Kp_linear 1.0

double angular_error_to_velocity(double error) {
    return Kp_angular * error;
}

double linear_error_to_velocity(double error) {
    return Kp_linear * error;
}