#include "motor_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <string.h>

#include "util.h"
#include "differential_drive.h"

#define MOTOR_DRIVER_QUEUE_SIZE 5
#define MOTOR_DRIVER_QUEUE_DELAY 10 // ticks

static QueueHandle_t xQueue = NULL;
static TickType_t delay = MOTOR_DRIVER_QUEUE_DELAY;
static void *msgin = NULL;

void push_to_motor_driver_queue(void *msgin) {
    differential_drive_motor_command_t * temp = msgin;
    printf("received msg! %f\n", temp->left_pwm_ratio);

    if (xQueue != NULL) {
        xQueueSendToBack(xQueue, &msgin, delay);
    }
}

void motor_driver_task(motor_driver_t *driver) {
    if (driver->init() != MOTOR_DRIVER_SUCCESS) {
        printf("Motor driver init failed. Exiting task...\n");
	vTaskDelete(NULL);
    }

    xQueue = xQueueCreate(MOTOR_DRIVER_QUEUE_SIZE, sizeof(void *));

    while (1) {
        if (xQueue == NULL) {
            // TODO: error handler
            continue;
        }

        if (xQueueReceive(xQueue, &msgin, delay) != pdPASS) {
            // TODO: error handler
            continue;
        }

        printf("received data!\n");

        differential_drive_motor_command_t * temp = msgin;
        printf("received msgaa! %f\n", temp->left_pwm_ratio);

        if (driver->handler(msgin) != MOTOR_DRIVER_SUCCESS) {
            // TODO: error handler
            continue;
        }
    }

    // clean up memory (if task ever reaches this point)
    free(msgin);
    free(driver);
    vTaskDelete(NULL);
}
