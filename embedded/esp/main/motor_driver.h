#pragma once

#include <drone_data/drone_data/msg/robot_position.h>

enum direction {
    FORWARD,
    REVERSE
};

struct motor_command {
    enum direction dir;
    double pwm_ratio;
};

struct queue_data {
    struct motor_command left;
    struct motor_command right;
};

void push_to_queue(struct queue_data d);
void motor_task(void *param);