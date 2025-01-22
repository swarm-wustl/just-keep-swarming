#include "motor_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "util.h"

#define MOTOR_DRIVER_QUEUE_SIZE 5
#define MOTOR_DRIVER_QUEUE_DELAY 10 // ticks

static QueueHandle_t xQueue = NULL;
static TickType_t delay = MOTOR_DRIVER_QUEUE_DELAY;
static void *msgin;

void push_to_motor_driver_queue(void *msgin) {
    if (xQueue != NULL) {
        xQueueSendToBack(xQueue, msgin, delay);
    }
}

void motor_driver_task(motor_driver_t *driver) {
    if (driver->init() != MOTOR_DRIVER_SUCCESS) {
        // TOOD: error handler
        printf("init failed!\n");
        return;
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

        if (driver->handler(msgin) != MOTOR_DRIVER_SUCCESS) {
            // TODO: error handler
            continue;
        }

        // clean up memory
        free(msgin);
    }
}
