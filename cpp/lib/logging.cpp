#include "logging.h"
#include <stdio.h>
#include <stdarg.h>
#include <string>

static int _MAX_LOG_LEVEL = 1;

void LogSetLevel(int level) {
  _MAX_LOG_LEVEL = level;
}

void LogAtLevel(int level, const char *label, const char *fmt, ...) {
  // printf("LOG LEVEL: %d, MY MAX: %d\n", level, _MAX_LOG_LEVEL);
  if (level > _MAX_LOG_LEVEL) return;
  va_list args;
  va_start(args, fmt);
  vprintf((std::string(label) + ": " + fmt + "\n").c_str(), args);
  va_end(args);
}
