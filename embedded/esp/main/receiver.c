#include "receiver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static QueueHandle_t xQueue = NULL;
static TickType_t delay = 10;
static struct queue_data data;

void push_to_queue(struct queue_data d) {
    // TODO: error handle
    if (xQueue != NULL) {
        xQueueSendToBack(xQueue, (void *)&d, delay);
    }
}

void receiving_task() {
    xQueue = xQueueCreate( 5, sizeof( struct queue_data ) );

    while (1) {
        if (xQueue != NULL) {
            if (xQueueReceive(xQueue, &data, delay) == pdPASS) {
                printf("received %d!\n", data.y);
            }
        }
    }
}

