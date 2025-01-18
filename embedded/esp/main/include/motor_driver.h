#pragma once

#include <drone_data/drone_data/msg/robot_position.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define MOTOR_QUEUE_SIZE 5
#define MOTOR_QUEUE_DELAY 10 // ticks

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 1000 // Hz

#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1

#define ENA GPIO_NUM_13
#define IN1 GPIO_NUM_12
#define IN2 GPIO_NUM_14
#define ENB GPIO_NUM_14   // TODO
#define IN3 GPIO_NUM_27
#define IN4 GPIO_NUM_26
#define GPIO_BITMASK (1ULL << ENA) | (1ULL << IN1) | (1ULL << IN2) | (1ULL << ENB) | (1ULL << IN3) | (1ULL << IN4)

#define PWM_STOP 0.0

typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} motor_direction_t;

void push_to_motor_driver_queue(void *msgin);
void motor_task(void *param);