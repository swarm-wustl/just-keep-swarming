#pragma once

#include <geometry_msgs/msg/twist.h>

typedef enum {
    COMMAND_PARSER_SUCCESS = 0,
    COMMAND_PARSER_ERROR_GENERIC = -128
} command_parser_ret_t;

// Generic command parser struct
// The implementation will be based on the type of robot (ex: diff drive, omnidirectional)
typedef command_parser_ret_t (*command_parser_handler_t)(const void *msgin, void *msgout);

typedef struct {
    const char *node_name;
    const char *subscriber_name;
    const rosidl_message_type_support_t *message_type;
    command_parser_handler_t handler;
} command_parser_t;

void command_parser_task(command_parser_t *parser);