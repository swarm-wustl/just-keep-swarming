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

    if (xQueue == NULL) {
        printf("xQueue init failed. Exiting task...\n");
        vTaskDelete(NULL); 
    }

    while (1) { 
        if (xQueueReceive(xQueue, &driver->msgin, delay) != pdPASS) {
            continue;
        }

        printf("Received data!\n");
        
        if (driver->handler(driver->msgin) != MOTOR_DRIVER_SUCCESS) {
            printf("Motor driver handler failed to parse message. Continuing task...\n");
            continue;
        }
    }

    // clean up memory (if task ever reaches this point)
    free(driver->msgin);

    vTaskDelete(NULL);
}
