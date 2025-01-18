#pragma once

#include <geometry_msgs/msg/twist.h>

typedef enum {
    COMMAND_PARSER_SUCCESS = 0,
    COMMAND_PARSER_ERROR_GENERIC = -128
} command_parser_ret_t;

// Generic command parser function
// The implementation will be based on the type of robot (ex: diff drive, omnidirectional)
typedef command_parser_ret_t (*command_parser_func_t)(void *msgin, void *msgout);

/*
TODO:
- Command parser task that receives ROS messages
- Make the task take an argument for the specific type it'll be handling as input (twist)
- Also make the task take an argument for the output type (diff drive commands)
- Use command/function ptr pattern to select parser within the executor callback
*/
// command_parser_func_t parser = twist_to_differential_drive;