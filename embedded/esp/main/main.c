#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "receiver.h"

static TaskHandle_t task_handle = NULL;

static void basic_task(void *param) {
    struct queue_data sending_data;

    sending_data.id = 1;
    sending_data.x = 25;
    sending_data.y = -30;

    while (1) {
        printf("sending task data in 3 seconds...\n");
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        push_to_queue(sending_data);
    }
}

void app_main(void)
{
    xTaskCreate(basic_task, "basic_task", 4*1024, NULL, 1, NULL);
    xTaskCreate(receiving_task, "receiving_task", 4*1024, NULL, 1, &task_handle);
}
