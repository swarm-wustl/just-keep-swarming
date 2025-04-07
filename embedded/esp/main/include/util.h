#pragma once

#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        printf("Failed status on line %d: %d. Aborting.\n",__LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
}

// please never forget to label HIGH and LOW as 1 and 0 respectively
enum digital_logic {
    HIGH = 1,
    LOW = 0
};