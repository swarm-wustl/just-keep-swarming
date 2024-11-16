#pragma once

#include <drone_data/drone_data/msg/robot_position.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define ENA GPIO_NUM_13
#define IN1 GPIO_NUM_12
#define IN2 GPIO_NUM_14
#define IN3 GPIO_NUM_27
#define IN4 GPIO_NUM_26
#define GPIO_BITMASK (1ULL << ENA) | (1ULL << IN1) | (1ULL << IN2) | (1ULL << IN3) | (1ULL << IN4)

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