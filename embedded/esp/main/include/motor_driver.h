#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} motor_direction_t;

typedef enum {
    MOTOR_DRIVER_SUCCESS = 0,
    MOTOR_DRIVER_ERROR_GENERIC = -128
} motor_driver_ret_t;

typedef struct {
    motor_driver_ret_t (*init)();
    motor_driver_ret_t (*handler)(void *msgin);
    void *msgin;
} motor_driver_t;

void push_to_motor_driver_queue(void *msgin);
void motor_driver_task(void *driver);