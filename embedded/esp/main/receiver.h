#pragma once

struct queue_data {
    unsigned long id;
    int x;
    int y;
};

void push_to_queue(struct queue_data d);
void receiving_task();