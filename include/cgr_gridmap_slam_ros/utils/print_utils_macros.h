//
// Created by kandithws on 7/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_PRINT_UTILS_MACRO_H_H
#define CGR_GRIDMAP_SLAM_ROS_PRINT_UTILS_MACRO_H_H

#include <stdio.h>
#include <cassert>

#define PREFIX_INFO "[LOG INFO]: "
#define PREFIX_WARN "[LOG WARN]: "
#define PREFIX_ERROR "[LOG ERROR]: "
#define PREFIX_DEBUG "[LOG DEBUG]: "
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m\n"

#define LOGPRINT_INFO(MSG, ...) printf(PREFIX_INFO MSG ANSI_COLOR_RESET, ##__VA_ARGS__)
#define LOGPRINT_WARN(MSG, ...) printf(ANSI_COLOR_YELLOW PREFIX_WARN MSG ANSI_COLOR_RESET, ##__VA_ARGS__)
#define LOGPRINT_ERROR(MSG, ...) printf(ANSI_COLOR_RED PREFIX_ERROR MSG ANSI_COLOR_RESET, ##__VA_ARGS__)
#define LOGPRINT_DEBUG(MSG, ...) printf(ANSI_COLOR_CYAN PREFIX_DEBUG MSG ANSI_COLOR_RESET, ##__VA_ARGS__)

#define LOGASSERT(expr) assert(expr)
#define LOGASSERT_MSG(expr, msg) assert((expr) && (msg))


#endif //CGR_GRIDMAP_SLAM_ROS_PRINT_UTILS_MACRO_H_H
