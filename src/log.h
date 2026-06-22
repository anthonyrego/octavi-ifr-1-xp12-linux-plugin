/* log.h - tiny logging wrapper around XPLMDebugString (writes to Log.txt). */
#ifndef OCTAVI_LOG_H
#define OCTAVI_LOG_H

#include <stdarg.h>
#include <stdio.h>
#include <XPLMUtilities.h>

#define OCTAVI_LOG_PREFIX "Octavi: "

static inline void octavi_log(const char *fmt, ...) {
  char msg[512];
  char out[600];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof msg, fmt, ap);
  va_end(ap);
  snprintf(out, sizeof out, OCTAVI_LOG_PREFIX "%s\n", msg);
  XPLMDebugString(out);
}

#endif /* OCTAVI_LOG_H */
