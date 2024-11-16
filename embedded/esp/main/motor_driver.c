#include "motor_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static QueueHandle_t xQueue = NULL;
static TickType_t delay = 10;
static struct queue_data data;

void push_to_motor_queue(struct queue_data d) {
    // TODO: error handle
    if (xQueue != NULL) {
        xQueueSendToBack(xQueue, (void *)&d, delay);
    }
}

void motor_task(void *param) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_BITMASK;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    xQueue = xQueueCreate(5, sizeof(struct queue_data));

    while (1) {
        if (xQueue != NULL) {
            if (xQueueReceive(xQueue, &data, delay) == pdPASS) {
                printf("received %d!\n", data.left.dir);

                // Turns motor 1 to max speed
                gpio_set_level(ENA, 1);

                // Sets motor 1 to the proper direction
                enum direction dir = data.left.dir;

                gpio_set_level(IN1, dir == FORWARD ? 1 : 0);
                gpio_set_level(IN2, dir == REVERSE ? 1 : 0);
            }
        }
    }
}

