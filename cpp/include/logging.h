#ifndef LOGGING_H_
#define LOGGING_H_

#include <stdarg.h>

void LogSetLevel(int level);
void LogAtLevel(int level, const char *label, const char *fmt, ...);

#define LOG_LEVEL_DEBUG (4)
#define LOG_LEVEL_INFO (3)
#define LOG_LEVEL_WARNING (2)
#define LOG_LEVEL_ERROR (1)
#define LOG_LEVEL_FATAL (0)
#define LOG_LEVEL_SUPPRESS (-1)

#define LOG_DEBUG(...) LogAtLevel(4, "DEBUG", __VA_ARGS__)
#define LOG_INFO(...) LogAtLevel(3, "INFO", __VA_ARGS__)
#define LOG_WARNING(...) LogAtLevel(2, "WARNING", __VA_ARGS__)
#define LOG_ERROR(...) LogAtLevel(1, "ERROR", __VA_ARGS__)
#define LOG_FATAL(...) LogAtLevel(0, "FATAL", __VA_ARGS__)

#endif  // LOGGING_H_
