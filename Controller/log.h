//
// Created by Hugo Trippaers on 24/07/2025.
//

#ifndef LOG_H
#define LOG_H
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
} log_level_t;

#ifndef LOG_DEFAULT_LEVEL
#define LOG_DEFAULT_LEVEL LOG_LEVEL_DEBUG
#endif

// Enable to print file and line for each log line
// #define LOG_WITH_FILE_AND_LINE

// Allow overriding
static int g_log_level = LOG_LEVEL_DEBUG;

static inline const char* log_level_str(log_level_t level) {
    switch((int)level) {
        case LOG_LEVEL_DEBUG: return "DEBUG";
        case LOG_LEVEL_INFO:  return "INFO";
        case LOG_LEVEL_WARN:  return "WARN";
        case LOG_LEVEL_ERROR: return "ERROR";
        case LOG_LEVEL_FATAL: return "FATAL";
        default:              return "UNKNOWN";
    }
}

#ifdef LOG_WITH_FILE_AND_LINE
#define log_printf(level, fmt, ...) \
do { \
if ((level) >= g_log_level) { \
printf("[%s:%d][%s] " fmt "\r\n", \
__FILE__, __LINE__, log_level_str(level), ##__VA_ARGS__); \
} \
} while(0)
#else
#define log_printf(level, fmt, ...) \
do { \
if ((level) >= g_log_level) { \
printf("[%s] " fmt "\n", \
log_level_str(level), ##__VA_ARGS__); \
} \
} while(0)
#endif


#define LOG_DEBUG(fmt, ...)   log_printf(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)    log_printf(LOG_LEVEL_INFO,  fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)    log_printf(LOG_LEVEL_WARN,  fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)   log_printf(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...)   log_printf(LOG_LEVEL_FATAL, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif
#endif //LOG_H
