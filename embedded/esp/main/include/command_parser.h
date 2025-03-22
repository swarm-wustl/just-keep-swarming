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
    const char *topic_name;
    const rosidl_message_type_support_t *message_type;
    command_parser_handler_t handler;

    /*
    * 
    * We have a few options for the message implementation:
    * 1) a void pointer: not bad but have to malloc memory
    * 2) a generic-sized static buffer: chill but limited to the same size for all data types
    * * (what if one possible type is really small, but another is huge?)
    * 3) tagged union: a more type-safe alternative to a void pointer
    * still kind of the same problem as (2) since it's limited to the largest possible size (unless we use pointers)
    * BUT if we don't use pointers we can take advantage of stack allocation
    * unions also make it difficult to use const, unless we're using a pointer
    * unions would also make implementation more complicated in this case
    * 
    */

    void * msgin;
    void * msgout;
} command_parser_t;

void command_parser_task(command_parser_t *parser);